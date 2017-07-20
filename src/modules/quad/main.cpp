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

static bool verbose = false;

//static HardwareTimer hwtimer;
class Timer {
 public:
  Timer() {
    Reset();
  }

  float GetElapsedSeconds() {
    return float(hrt_absolute_time() - _t0) / 1e-6f;
  }

  uint64_t GetElapsedMicroSeconds() {
    return hrt_absolute_time() - _t0;
  }

  void Reset() {
    _t0 = hrt_absolute_time();
  }

 private:
  uint64_t _t0;
};

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

// Telemetry
  TelemetryPacket::data_packet_t dataPacket1, dataPacket2;

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
      //TODO MWM: decode data, input here.
//      mlInputs.joystickInput.axisLeftHorizontal =
      mlInputs.joystickInput.updated = true;
//      logic.SetRadioMessage(RadioTypes::RadioMessageDecoded(raw.data));
    } else {
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
    actuatorCmds.values[0] = out.motorCommand1;
    actuatorCmds.values[1] = out.motorCommand2;
    actuatorCmds.values[2] = out.motorCommand3;
    actuatorCmds.values[3] = out.motorCommand4;

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

    int quadType = 0;
    if (0 != param_get(param_find("QUADCOPTER_TYPE"), &quadType)) {
      printf("Failed to get param <QUADCOPTER_TYPE>\n");
      quadType = 0;
    }

    int vehId = 0;
    if (0 != param_get(param_find("VEHICLE_ID"), &vehId)) {
      printf("Failed to get param <VEHICLE_ID>\n");
      vehId = 0;
    }

//    logic.Initialise(Onboard::QuadcopterConstants::QuadcopterType(quadType),
//                     vehId);

    daemon_task = px4_task_spawn_cmd("logic_quad", SCHED_DEFAULT,
    SCHED_PRIORITY_DEFAULT,
                                     4096, logicThread, (char * const *) NULL);

    printf("Started\n");
    return 0;
  }

  if (!strcmp(argv[1], "go")) {
    if (!thread_running) {
      printf("Thread not running\n");
      return 0;
    }

    int takeoffDelay = 3;
    if (argc > 2) {
      takeoffDelay = atoi(argv[2]);
    }
    printf("Will take off in %d seconds:\n", takeoffDelay);
    for (int i = 0; i < takeoffDelay; i++) {
      printf("%d\n", int(takeoffDelay - i));
      usleep(1000 * 1000);
    }
    printf("Decolage!\n");
//    logic.SetGoAutonomous();
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

  if (!strcmp(argv[1], "test") || !strcmp(argv[1], "t")) {
    if (argc < 3) {
      usage();
      return -1;
    }

    if (!strcmp(argv[2], "thrust") || !strcmp(argv[2], "t")) {
      if (argc < 4) {
        usage();
        return -1;
      }
      printf("argc=%d\n", int(argc));
      const unsigned TEST_TIME = 5;
      float thrustFrac = atof(argv[3]);
      printf("Running thrust test for %ds, @ %.3f*weight\n", TEST_TIME,
             double(thrustFrac));
      //TODO: test modes
//      logic.TestMotors(true, thrustFrac);
      usleep(1 * 1000 * 1000);
//      logic.PrintStatus();
      usleep((TEST_TIME - 1) * 1000 * 1000);
//      logic.TestMotors(false, 0);
    }
  }

  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
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
        printf("  orb_sub_rangeSensorReport  = %d\n", orb_sub_rangeSensorReport);
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
