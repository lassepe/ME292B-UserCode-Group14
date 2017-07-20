/* Motor driver for crazyflie
 * Super non-portable!
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <poll.h>

#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>

#include <pthread.h>

#include "drivers/drv_pwm_output.h"
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_direct.h>
#include <uORB/topics/battery_status.h>

#include <systemlib/param/param.h>

static volatile bool thread_running = false; /**< Daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */

extern "C" __EXPORT int cf_motors_main(int argc, char *argv[]);
int testMotors();
int cf_motors_thread(int argc, char *argv[]);

const unsigned LOOP_FREQUENCY = 1000;  //Hz

volatile unsigned numTimeouts = 0;
volatile unsigned numCmdsReceived = 0;

static int orb_sub_actuator_direct = 0;
static int orb_sub_battery = 0;

volatile float batt_v = 4.1;
float const battFiltConst = 0.01;

static void usage() {
  printf("usage:\n");
  printf("Normal operation: cf_motors {start|stop|status|test}\n");
  printf("\tTest operation: cf_motors test\n");
  printf("\n");
  return;
}

unsigned const TEST_MOTOR_COMMAND = 25;
int testMotors() {
  orb_sub_battery = orb_subscribe(ORB_ID(battery_status));
  printf("orb_sub_battery  = %d\n", orb_sub_battery);

  const uint32_t updatePeriod_us = 1000000 / LOOP_FREQUENCY;
  printf("sleep  = %d\n", int(updatePeriod_us));

  /* open for ioctl only */
  const char *dev = PWM_OUTPUT0_DEVICE_PATH;
  int fd = open(dev, 0);

  if (fd < 0) {
    printf("can't open %s, err = %d\n", dev, fd);
    return -1;
  }

  int ret1 = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
  int ret2 = ioctl(fd, PWM_SERVO_ARM, 0);
  printf("Arming = %d, %d\n", ret1, ret2);

  const float RUN_TIME = 5.0f;
  struct battery_status_s batt;

  unsigned const printPeriod_us = 500000;
  unsigned nextPrintTime = 0;
  unsigned numBattMeas = 0;

  uint64_t t0 = hrt_absolute_time();

  while (true ) {
    uint64_t tElapsed_us = hrt_absolute_time() - t0;  //[us]
    if (tElapsed_us >= (RUN_TIME * 1000000)) {
      break;
    }
    bool updated = false;
    orb_check(orb_sub_battery, &updated);
    if (updated) {
      numBattMeas++;
      orb_copy(ORB_ID(battery_status), orb_sub_battery, &batt);
      //update battery voltage estimate:
      batt_v = (1 - battFiltConst) * batt_v + battFiltConst * batt.voltage_v;
    }

    if (tElapsed_us > nextPrintTime) {
      nextPrintTime += printPeriod_us;
      printf("batt_v = %.3f (over %d meas)\n", double(batt_v),
             int(numBattMeas));
    }

    for (int i = 0; i < 4; i++) {
      int ret = ioctl(fd, PWM_SERVO_SET(i), TEST_MOTOR_COMMAND);

      if (ret != OK) {
        err(1, "PWM_SERVO_SET(%d) = %d (%.3f)", i, TEST_MOTOR_COMMAND,
            double(TEST_MOTOR_COMMAND));
      }
    }

    usleep(updatePeriod_us);
  }

  //Make sure we exit with motors off:
  for (int i = 0; i < 4; i++) {
    int ret = ioctl(fd, PWM_SERVO_SET(i), 0);

    if (ret != OK) {
      err(1, "PWM_SERVO_SET(%d)", i);
    }
  }

  ioctl(fd, PWM_SERVO_DISARM, 0);

  return 0;
}

int cf_motors_thread(int argc, char *argv[]) {
  orb_sub_actuator_direct = orb_subscribe(ORB_ID(actuator_direct));
  orb_sub_battery = orb_subscribe(ORB_ID(battery_status));

  if (orb_sub_actuator_direct <= 0) {
    printf("Error in orb_sub_actuator_direct  = %d\n", orb_sub_actuator_direct);
    return -1;
  }

  const uint32_t updatePeriod_us = 1000000 / LOOP_FREQUENCY;
  printf("sleep  = %d\n", int(updatePeriod_us));

  /* open for ioctl only */
  const char *dev = PWM_OUTPUT0_DEVICE_PATH;
  int fd = open(dev, 0);

  if (fd < 0) {
    printf("can't open %s, err = %d\n", dev, fd);
    return -1;
  }

  int ret1 = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
  int ret2 = ioctl(fd, PWM_SERVO_ARM, 0);
  printf("Arming = %d, %d\n", ret1, ret2);

  px4_pollfd_struct_t fds[2];
  fds[0].fd = orb_sub_actuator_direct;
  fds[0].events = POLLIN;

  struct battery_status_s batt;
  struct actuator_direct_s actuatorCmds;

  bool motorsOff = true;
  const unsigned MOTOR_TIMEOUT_ms = 50;  //if we don't receive a command every this often, kill motors.

  while (thread_running) {
    int poll_ret = px4_poll(fds, 1, MOTOR_TIMEOUT_ms);
    if (poll_ret == 0) {
      numTimeouts++;
      if (motorsOff) {
        //nothing to be done.
        continue;
      }
      //time-out, but motors' last command wasn't zero!
      //PANIC!
      thread_running = false;
      motorsOff = true;
      printf("Timeout panic! Killing motors\n");
      break;
    }

    if (fds[0].revents & POLLIN) {
      /* obtained data for the first file descriptor */
      /* copy sensors raw data into local buffer */

      bool updated = false;
      orb_check(orb_sub_battery, &updated);
      if (updated) {
        /* obtained data for the battery*/
        orb_copy(ORB_ID(battery_status), orb_sub_battery, &batt);
        //update battery voltage estimate:
        batt_v = (1 - battFiltConst) * batt_v + battFiltConst * batt.voltage_v;
      }

      numCmdsReceived++;
      orb_copy(ORB_ID(actuator_direct), orb_sub_actuator_direct, &actuatorCmds);
      if (actuatorCmds.nvalues == 0 && motorsOff) {
        //empty command, most likely the first we get. Skip it.
        continue;
      }
      if (actuatorCmds.nvalues != 4) {
        //Incorrect number of motors!
        err(1, "PWM: incorrect number of motors! %d", actuatorCmds.nvalues);
        thread_running = false;
        break;
      }

      for (int i = 0; i < 4; i++) {
        if (actuatorCmds.values[i] > 0) {
          motorsOff = false;
        }

        int const pwmVal = actuatorCmds.values[i];
        int ret = ioctl(fd, PWM_SERVO_SET(i), pwmVal);

        if (ret != OK) {
          err(1, "PWM_SERVO_SET(%d) = %d (%.3f)", i, pwmVal,
              double(actuatorCmds.values[i]));
        }
      }
    }

    usleep(updatePeriod_us);  //TODO THis isn't very nice, there will be jitter between logic creating motor commands, and them beign executed.
  }

  //Make sure we exit with motors off:
  for (int i = 0; i < 4; i++) {
    int ret = ioctl(fd, PWM_SERVO_SET(i), 0);

    if (ret != OK) {
      err(1, "PWM_SERVO_SET(%d)", i);
    }
  }

  ioctl(fd, PWM_SERVO_DISARM, 0);

  printf("Exiting: cf_motors_thread\n");

  return 0;
}

int cf_motors_main(int argc, char *argv[]) {

  if (argc < 1) {
    usage();
    return -1;
  }

  if (!strcmp(argv[1], "test")) {
    if (thread_running) {
      printf("A thread already running\n");
      return -1;
    }

    testMotors();

    usleep(100 * 1000);

    return 0;
  }

  if (!strcmp(argv[1], "setMotorType")) {
    if (thread_running) {
      printf("Thread already running\n");
      return 0;
    }

    if (argc < 3) {
      printf("Too few arguments\n");
      return 0;
    }

    int motorTypeIn = atoi(argv[2]);

    if (param_set(param_find("MOTOR_TYPE"), &motorTypeIn)) {
      printf("Failed to set <MOTOR_TYPE>\n");
    } else {
      printf("Param set: MOTOR_TYPE to %d\n", motorTypeIn);
      printf("Now you *MUST* run `param save` to store the params\n");
    }

    return 0;
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      printf("Thread already running\n");
      return 0;
    }

    usleep(100000);

    thread_running = true;

    daemon_task = px4_task_spawn_cmd(
        "cf_motors", SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT + 5,
        2000, cf_motors_thread,
        (argv) ? (char * const *) &argv[2] : (char * const *) NULL);

    printf("Started\n");
    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    if (!thread_running) {
      printf("Nothing to do...\n");
      return 0;
    }

    thread_running = false;

    printf("Stopped!\n");
    return 0;
  }

  if (!strcmp(argv[1], "test") || !strcmp(argv[1], "t")) {
    if (argc < 2) {
      usage();
      return -1;
    }

    //TODO: test modes
  }

  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      printf("\tmotors is running.\n");
      printf("\tNum timeouts = %d, num cmds = %d\n", numTimeouts,
             numCmdsReceived);
    } else {
      printf("\tmotors not started\n");
    }
    printf("\tVoltage = %.3fV\n", double(batt_v));
    return 0;
  }

  usage();
  return -1;
}
