/* Quadcopter app, modified from lab code for ME136
 *
 * (c) 2017 Muellerlab, UC Berkeley.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <nuttx/sched.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/actuator_direct.h>
#include <uORB/topics/radio_received.h>
#include <uORB/topics/radio_send.h>
#include <uORB/topics/radio_send_ready.h>
#include <uORB/topics/optical_flow_report.h>
#include <uORB/topics/range_sensor_report.h>

#include "TelemetryPacket.hpp"
#include "RadioTypes.hpp"

#include "Vec3f.hpp"
#include "MainLoopTypes.hpp"
#include "UserCode.hpp"

static volatile bool thread_running = false; /**< Daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */
static volatile bool syslink_ready = false;

extern "C" __EXPORT int quad_main(int argc, char *argv[]);
void OnTimer(void* p);
int logicThread(int argc, char *argv[]);

const unsigned ONBOARD_FREQUENCY = 500;  //Hz

int teamID = 0;

static bool verbose = false;

void OnTimer(void* p) {
  sem_t* _sem = (sem_t*) p;
  int svalue;
  sem_getvalue(_sem, &svalue);
  if (svalue < 0)
    sem_post(_sem);
}

static void usage() {
  printf("usage:\n");
  printf("Normal operation: quad {start|stop|status|debug}\n");
  printf("\n");
  return;
}

//UORB STUFF
static orb_advert_t orb_pub_actuatorCmds = 0;
static orb_advert_t orb_pub_radioSend = 0;
static struct actuator_direct_s actuatorCmds;
static struct radio_send_s radio_send;
static int orb_sub_battery = 0;
static int orb_sub_imuAccel = 0;
static int orb_sub_imuGyro = 0;
static int orb_sub_radioReceived = 0;
static int orb_sub_radioSendReady = 0;
static int orb_sub_flowReport = 0;
static int orb_sub_rangeSensorReport = 0;

static unsigned count_battery = 0;
static unsigned count_imuAccel = 0;
static unsigned count_imuGyro = 0;
static unsigned count_radioReceived = 0;
static unsigned count_radioSendReady = 0;
static unsigned count_flowReport = 0;
static unsigned count_rangeSensorReport = 0;

static bool armed = false;
static unsigned cyclesSinceRadioCommand = 0;

int logicThread(int argc, char *argv[]) {
  if (!orb_pub_actuatorCmds) {
    memset(&actuatorCmds, 0, sizeof(actuatorCmds));
    orb_pub_actuatorCmds = orb_advertise(ORB_ID(actuator_direct),
                                         &actuatorCmds);
  }

  if (!orb_pub_radioSend) {
    memset(&radio_send, 0, sizeof(radio_send));
    orb_pub_radioSend = orb_advertise(ORB_ID(radio_send), &radio_send);
  }

  if (!orb_sub_battery) {
    orb_sub_battery = orb_subscribe(ORB_ID(battery_status));
  }

  if (!orb_sub_imuAccel) {
    orb_sub_imuAccel = orb_subscribe(ORB_ID(sensor_accel));
  }

  if (!orb_sub_imuGyro) {
    orb_sub_imuGyro = orb_subscribe(ORB_ID(sensor_gyro));
  }

  if (!orb_sub_radioReceived) {
    orb_sub_radioReceived = orb_subscribe(ORB_ID(radio_received));
  }

  if (!orb_sub_radioSendReady) {
    orb_sub_radioSendReady = orb_subscribe(ORB_ID(radio_send_ready));
  }

  if (!orb_sub_flowReport) {
    orb_sub_flowReport = orb_subscribe(ORB_ID(optical_flow_report));
  }

  if (!orb_sub_rangeSensorReport) {
    orb_sub_rangeSensorReport = orb_subscribe(ORB_ID(range_sensor_report));
  }

  static struct hrt_call ol_tick_call;
  memset(&ol_tick_call, 0, sizeof(ol_tick_call));

  printf("Started: fast_loop...\n");

  sem_t _sem;
  sem_init(&_sem, 0, 0);

  const hrt_abstime updatePeriod = 1000000 / ONBOARD_FREQUENCY;
  hrt_call_every(&ol_tick_call, updatePeriod, updatePeriod, &OnTimer, &_sem);

  perf_counter_t perf_timer_obLogic;
  perf_timer_obLogic = perf_alloc(PC_ELAPSED, "quad_logic_run");

  //Load the accelerometer corrections:
  struct {
    Vec3f b;
    Vec3f k;
  } accCorr;

  bool loadSuccess = true;
  if (0 != param_get(param_find("CALIBACC_BX"), &accCorr.b.x)) {
    loadSuccess = false;
    printf("Failed to get param <CALIBACC_BX>\n");
  }
  if (0 != param_get(param_find("CALIBACC_BY"), &accCorr.b.y)) {
    loadSuccess = false;
    printf("Failed to get param <CALIBACC_BY>\n");
  }
  if (0 != param_get(param_find("CALIBACC_BZ"), &accCorr.b.z)) {
    loadSuccess = false;
    printf("Failed to get param <CALIBACC_BZ>\n");
  }
  if (0 != param_get(param_find("CALIBACC_KX"), &accCorr.k.x)) {
    loadSuccess = false;
    printf("Failed to get param <CALIBACC_KX>\n");
  }
  if (0 != param_get(param_find("CALIBACC_KY"), &accCorr.k.y)) {
    loadSuccess = false;
    printf("Failed to get param <CALIBACC_KY>\n");
  }
  if (0 != param_get(param_find("CALIBACC_KZ"), &accCorr.k.z)) {
    loadSuccess = false;
    printf("Failed to get param <CALIBACC_KZ>\n");
  }

  if (!loadSuccess) {
    printf("Failed to load params!\n");
    return -1;
  }

  battery_status_s batt;
  sensor_accel_s acc;
  sensor_gyro_s gyro;
  radio_send_ready_s rsReady;

  /* Check syslink_ready here since syslink publishes ready status on startup,
   but checking if the topic has been updated below will yield false */
  if (!syslink_ready) {
    memset(&rsReady, 0, sizeof(rsReady));
    orb_copy(ORB_ID(radio_send_ready), orb_sub_radioSendReady, &rsReady);
    syslink_ready = rsReady.is_ready;
  }

  MainLoopInput mlInputs;
  mlInputs.batteryVoltage.value = 0;
  mlInputs.batteryVoltage.updated = false;

  mlInputs.currentTime = 0;

  mlInputs.heightSensor.value = 0;
  mlInputs.heightSensor.updated = false;

  mlInputs.imuMeasurement.accelerometer = Vec3f(0, 0, 0);
  mlInputs.imuMeasurement.rateGyro = Vec3f(0, 0, 0);
  mlInputs.imuMeasurement.updated = false;

  mlInputs.joystickInput.axisLeftHorizontal = 0;
  mlInputs.joystickInput.axisLeftVertical = 0;
  mlInputs.joystickInput.axisRightHorizontal = 0;
  mlInputs.joystickInput.axisRightVertical = 0;
  mlInputs.joystickInput.buttonBlue = false;
  mlInputs.joystickInput.buttonGreen = false;
  mlInputs.joystickInput.buttonRed = false;
  mlInputs.joystickInput.buttonYellow = false;
  mlInputs.joystickInput.buttonStart = false;
  mlInputs.joystickInput.updated = false;

  mlInputs.opticalFlowSensor.value_x = 0;
  mlInputs.opticalFlowSensor.value_y = 0;
  mlInputs.opticalFlowSensor.updated = false;

  //whether we allow the motors to turn
  armed = false;
  const unsigned TIMEOUT_ARMED_CYCLES = unsigned(0.2 * ONBOARD_FREQUENCY);  //in cycles

  bool updated;
  thread_running = true;
  while (thread_running) {
    sem_wait(&_sem);
    perf_begin(perf_timer_obLogic);

    updated = false;
    orb_check(orb_sub_battery, &updated);
    if (updated) {
      count_battery++;
      orb_copy(ORB_ID(battery_status), orb_sub_battery, &batt);
      mlInputs.batteryVoltage.value = batt.voltage_v;
      mlInputs.batteryVoltage.updated = true;
    } else {
      mlInputs.batteryVoltage.updated = false;
    }

    updated = false;
    orb_check(orb_sub_imuAccel, &updated);
    if (updated) {
      count_imuAccel++;
      orb_copy(ORB_ID(sensor_accel), orb_sub_imuAccel, &acc);
      mlInputs.imuMeasurement.updated = true;
      mlInputs.imuMeasurement.accelerometer = Vec3f(
          (1 / accCorr.k.x) * (acc.x - accCorr.b.x),
          (1 / accCorr.k.y) * (acc.y - accCorr.b.y),
          (1 / accCorr.k.z) * (acc.z - accCorr.b.z));
    } else {
      mlInputs.imuMeasurement.updated = false;
    }

    updated = false;
    orb_check(orb_sub_imuGyro, &updated);
    if (updated) {
      count_imuGyro++;
      orb_copy(ORB_ID(sensor_gyro), orb_sub_imuGyro, &gyro);
      mlInputs.imuMeasurement.updated = true;
      mlInputs.imuMeasurement.rateGyro = Vec3f(gyro.x, gyro.y, gyro.z);
    } else {
      mlInputs.imuMeasurement.updated = false;
      //note, this means we only call it "updated" if gyro updates.
    }

    updated = false;
    orb_check(orb_sub_radioReceived, &updated);
    if (updated) {
      count_radioReceived++;
      struct radio_received_s raw;
      memset(&raw, 0, sizeof(raw));
      orb_copy(ORB_ID(radio_received), orb_sub_radioReceived, &raw);
      RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
          raw.data);

      mlInputs.joystickInput.axisLeftHorizontal = msg.floats[0];
      mlInputs.joystickInput.axisLeftVertical = msg.floats[1];
      mlInputs.joystickInput.axisRightHorizontal = msg.floats[2];
      mlInputs.joystickInput.axisRightVertical = msg.floats[3];

      mlInputs.joystickInput.buttonRed = msg.buttonRed;
      mlInputs.joystickInput.buttonGreen = msg.buttonGreen;
      mlInputs.joystickInput.buttonBlue = msg.buttonBlue;
      mlInputs.joystickInput.buttonYellow = msg.buttonYellow;
      mlInputs.joystickInput.buttonStart = msg.buttonStart;
      mlInputs.joystickInput.buttonSelect = msg.buttonSelect;

      mlInputs.joystickInput.updated = true;

      cyclesSinceRadioCommand = 0;
      if (msg.flags) {
        armed = true;
      } else {
        armed = false;
      }
    } else {
      cyclesSinceRadioCommand++;
      mlInputs.joystickInput.updated = false;
    }

    updated = false;
    orb_check(orb_sub_flowReport, &updated);
    if (updated) {
      count_flowReport++;
      struct optical_flow_report_s flowReport;
      orb_copy(ORB_ID(optical_flow_report), orb_sub_flowReport, &flowReport);
      mlInputs.opticalFlowSensor.value_x = flowReport.flow_x;
      mlInputs.opticalFlowSensor.value_y = flowReport.flow_y;
      mlInputs.opticalFlowSensor.updated = true;
    } else {
      mlInputs.opticalFlowSensor.updated = false;
    }

    updated = false;
    orb_check(orb_sub_rangeSensorReport, &updated);
    if (updated) {
      count_rangeSensorReport++;
      struct range_sensor_report_s rangeReport;
      orb_copy(ORB_ID(range_sensor_report), orb_sub_rangeSensorReport,
               &rangeReport);
      mlInputs.heightSensor.value = rangeReport.range;
      mlInputs.heightSensor.updated = true;
    } else {
      mlInputs.heightSensor.updated = false;
    }

    MainLoopOutput out = MainLoop(mlInputs);

    //collect outputs:
    actuatorCmds.nvalues = 4;
    //Note, we reshuffle the indices
    if (cyclesSinceRadioCommand > TIMEOUT_ARMED_CYCLES) {
      //timeout!
      armed = false;
    }
    if (armed) {
      actuatorCmds.values[0] = out.motorCommand2;  //+x, -y
      actuatorCmds.values[1] = out.motorCommand4;  //-x, +y
      actuatorCmds.values[2] = out.motorCommand1;  //+x, +y
      actuatorCmds.values[3] = out.motorCommand3;  //-x, -y
    } else {
      //ignore commands!
      actuatorCmds.values[0] = 0;
      actuatorCmds.values[1] = 0;
      actuatorCmds.values[2] = 0;
      actuatorCmds.values[3] = 0;
    }

    // TODO: Telemetry

//    logic.GetTelemetryDataPackets(dataPacket1, dataPacket2);

    //do the uORB stuff
    bool orbErr = false;
    orbErr |= orb_publish(ORB_ID(actuator_direct), orb_pub_actuatorCmds,
                          &actuatorCmds);

    // Telemetry
    updated = false;
    orb_check(orb_sub_radioSendReady, &updated);
    if (updated) {
      count_radioSendReady++;
      memset(&rsReady, 0, sizeof(rsReady));
      orb_copy(ORB_ID(radio_send_ready), orb_sub_radioSendReady, &rsReady);
      syslink_ready = rsReady.is_ready;
    }
    if (syslink_ready) {
      /* numPackets = number of packets to be grouped together in a single transmission
       For the telemetry, we send two packets. The first contains imu data (accel and gyro)
       and the second contains estimator states. Packets are grouped together to ensure that
       the uwb radio ATTEMPTS to send the whole group one after the other.
       Note: this does not ensure receival of the entire group, as individual packets can still
       get dropped. */

      static uint8_t telPacketCounter = 0;
      telPacketCounter = (telPacketCounter + 1) % 256;

      TelemetryPacket::data_packet_t dataPacket1, dataPacket2;

      TelemetryPacket::TelemetryPacket pkt;
      pkt.packetNumber = telPacketCounter;
      pkt.accel[0] = mlInputs.imuMeasurement.accelerometer.x;
      pkt.accel[1] = mlInputs.imuMeasurement.accelerometer.y;
      pkt.accel[2] = mlInputs.imuMeasurement.accelerometer.z;
      pkt.gyro[0] = mlInputs.imuMeasurement.rateGyro.x;
      pkt.gyro[1] = mlInputs.imuMeasurement.rateGyro.y;
      pkt.gyro[2] = mlInputs.imuMeasurement.rateGyro.z;
      pkt.battVoltage = mlInputs.batteryVoltage.value;
      for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS;
          i++) {
        pkt.debugVals[i] = out.telemetryOutputs_plusMinus100[i];
      }
      pkt.debugchar = out.telemetryOutputDebugChar;
      pkt.heightsensor = mlInputs.heightSensor.value;
      pkt.motorCmds[0] = out.motorCommand1;
      pkt.motorCmds[1] = out.motorCommand2;
      pkt.motorCmds[2] = out.motorCommand3;
      pkt.motorCmds[3] = out.motorCommand4;
      pkt.opticalFlowx = float(mlInputs.opticalFlowSensor.value_x);
      pkt.opticalFlowy = float(mlInputs.opticalFlowSensor.value_y);

      pkt.type = TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT1;
      EncodeTelemetryPacket(pkt, dataPacket1);
      pkt.type = TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT2;
      EncodeTelemetryPacket(pkt, dataPacket2);

      radio_send.numPackets = 2;
      memset(radio_send.data1, 0, sizeof(radio_send.data1));
      memcpy(radio_send.data1, &dataPacket1,
             sizeof(TelemetryPacket::data_packet_t));
      memset(radio_send.data2, 0, sizeof(radio_send.data2));
      memcpy(radio_send.data2, &dataPacket2,
             sizeof(TelemetryPacket::data_packet_t));
      orbErr |= orb_publish(ORB_ID(radio_send), orb_pub_radioSend, &radio_send);
    }

    if (orbErr) {
      thread_running = false;
      printf("Error publishing to uorb!\n");
      continue;
    }

    perf_end(perf_timer_obLogic);
  }

  hrt_cancel(&ol_tick_call);
  usleep(100);
  sem_destroy(&_sem);

  printf("Exiting: fast_loop\n");

  return 0;
}

int quad_main(int argc, char *argv[]) {

  if (argc < 2) {
    usage();
    return -1;
  }

  if (!strcmp(argv[1], "setTeamId")) {
    if (argc >= 3) {
      int vehIdIn = atoi(argv[2]);
      if ((vehIdIn >= 1) && (vehIdIn <= 255)) {
        //valid argument
        printf("setting vehicle ID to %d\n", vehIdIn);

        if (param_set(param_find("VEHICLE_ID"), &vehIdIn)) {
          printf("Failed to set parameter\n");
        } else {
          printf("Param set: to %d\n", vehIdIn);
          printf("Now you *MUST* run `param save` to store the params\n");
        }
        return 0;
      } else {
        printf("Invalid type\n");
      }
    } else {
      printf("Too few arguments\n");
    }

    printf("e.g. quad setTeamId 7\n");
    return -1;
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      printf("Thread already running\n");
      return 0;
    }

    if (argc > 2) {
      if (!strcmp(argv[2], "v")) {
        verbose = true;
        printf("Setting to verbose mode\n");
      }
    }

    if (0 != param_get(param_find("VEHICLE_ID"), &teamID)) {
      printf("Failed to get param <VEHICLE_ID>\n");
      teamID = 0;
    }

    daemon_task = px4_task_spawn_cmd("logic_quad", SCHED_DEFAULT,
    SCHED_PRIORITY_DEFAULT,
                                     4096, logicThread, (char * const *) NULL);

    printf("Started\n");
    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    if (!thread_running) {
      printf("thread not running\n");
      return 0;
    }
    thread_running = false;

    usleep(1000);
    OnTimer(nullptr);

    printf("done!\n");
    return 0;
  }

  if (!strcmp(argv[1], "status")) {
    if (teamID == 0) {
      printf("ERROR!\nERROR!\nERROR!\nERROR!\n");
      printf("You have an invalid team id set. \n");
      printf(
          "Run the following, but replace `7` with your team's ID. Then turn the vehicle off and on.\n");
      printf(" quad setTeamId 7\n\n");
    }
    if (thread_running) {
      printf("STATUS: \n Team ID = %d\n", teamID);
      if (armed) {
        printf("vehicle is ARMED\n");
      } else {
        printf("vehicle is not armed\n");
      }
      printf("Cycles since last cmd = %ds\n", cyclesSinceRadioCommand);
      PrintStatus();
    } else {
      printf("\tquad not running\n");
    }
    return 0;
  }

  if (!strcmp(argv[1], "debug")) {
    if (thread_running) {
      printf("\n\n==== DEBUG ====\n");
      printf("Orb sub problems (should print no orbs if all OK).\n");
      if (!orb_sub_battery) {
        printf("  orb_sub_battery  = %d\n", orb_sub_battery);
      }
      if (!orb_sub_imuAccel) {
        printf("  orb_sub_imuAccel  = %d\n", orb_sub_imuAccel);
      }
      if (!orb_sub_imuGyro) {
        printf("  orb_sub_imuGyro  = %d\n", orb_sub_imuGyro);
      }
      if (!orb_sub_radioReceived) {
        printf("  orb_sub_radioReceived  = %d\n", orb_sub_radioReceived);
      }
      if (!orb_sub_radioSendReady) {
        printf("  orb_sub_radioSendReady  = %d\n", orb_sub_radioSendReady);
      }
      if (!orb_sub_flowReport) {
        printf("  orb_sub_flowReport  = %d\n", orb_sub_flowReport);
      }
      if (!orb_sub_rangeSensorReport) {
        printf("  orb_sub_rangeSensorReport  = %d\n",
               orb_sub_rangeSensorReport);
      }
      printf("End orb sub problems.\n");
      printf("-----\n");
      printf("Update counts (should be positive if receiving data):\n");
      printf("  count_battery = %d\n", count_battery);
      printf("  count_imuAccel = %d\n", count_imuAccel);
      printf("  count_imuGyro = %d\n", count_imuGyro);
      printf("  count_radioReceived = %d\n", count_radioReceived);
      printf("  count_radioSendReady = %d\n", count_radioSendReady);
      printf("  count_flowReport = %d\n", count_flowReport);
      printf("  count_rangeSensorReport = %d\n", count_rangeSensorReport);
    } else {
      printf("\tquad not running\n");
    }
    return 0;
  }

  printf("Unknown command\n");
  usage();
  return -1;
}
