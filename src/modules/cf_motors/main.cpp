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
int testMotors(float desiredSpeed[4]);
int cf_motors_thread(int argc, char *argv[]);
int pwm_value_from_speed(const float speed, float battV);

const unsigned LOOP_FREQUENCY = 1000;  //Hz

volatile unsigned numTimeouts = 0;
volatile unsigned numCmdsReceived = 0;

static int orb_sub_actuator_direct = 0;
static int orb_sub_battery = 0;

volatile float batt_v = 4.1;
float const battFiltConst = 0.01;

//Motor map constants:
enum {
	MOTOR_INVALID = 0, MOTOR_TEST_CMD_FEEDTHROUGH = 1, //assume that the input is a PWM value
	MOTOR_CF_STANDARD = 2,  //standard crazyflie motors & props
	MOTOR_BIGGER_PARROT_PROPS = 3,  //bigger motors, parrot props
	NUM_MOTOR_TYPES,
};

int currentMotorType = MOTOR_INVALID;

float mapConst_k_1_0[NUM_MOTOR_TYPES];
float mapConst_k_1_1[NUM_MOTOR_TYPES];
float mapConst_k_2_0[NUM_MOTOR_TYPES];
float mapConst_k_2_1[NUM_MOTOR_TYPES];
float mapConst_k_3_0[NUM_MOTOR_TYPES];
float mapConst_k_3_1[NUM_MOTOR_TYPES];

void initConstants();

void initConstants() {
	//Populate the constants:
	for (int i = 0; i < NUM_MOTOR_TYPES; i++) {
		mapConst_k_1_0[i] = 0.0f;
		mapConst_k_1_1[i] = 0.0f;
		mapConst_k_2_0[i] = 0.0f;
		mapConst_k_2_1[i] = 0.0f;
		mapConst_k_3_0[i] = 0.0f;
		mapConst_k_3_1[i] = 0.0f;
	}

	mapConst_k_1_0[MOTOR_TEST_CMD_FEEDTHROUGH] = 0.0f;
	mapConst_k_1_1[MOTOR_TEST_CMD_FEEDTHROUGH] = 0.0f;
	mapConst_k_2_0[MOTOR_TEST_CMD_FEEDTHROUGH] = 1.0f;
	mapConst_k_2_1[MOTOR_TEST_CMD_FEEDTHROUGH] = 0.0f;
	mapConst_k_3_0[MOTOR_TEST_CMD_FEEDTHROUGH] = 0.0f;
	mapConst_k_3_1[MOTOR_TEST_CMD_FEEDTHROUGH] = 0.0f;

	mapConst_k_1_0[MOTOR_CF_STANDARD] = -86.19993685f;
	mapConst_k_1_1[MOTOR_CF_STANDARD] = 22.87189816f;
	mapConst_k_2_0[MOTOR_CF_STANDARD] = 0.30208677f;
	mapConst_k_2_1[MOTOR_CF_STANDARD] = -0.07345602f;
	mapConst_k_3_0[MOTOR_CF_STANDARD] = -1.59346434e-05f;
	mapConst_k_3_1[MOTOR_CF_STANDARD] = 1.53209239e-05f;

	mapConst_k_1_0[MOTOR_BIGGER_PARROT_PROPS] = -379.31113434f;
	mapConst_k_1_1[MOTOR_BIGGER_PARROT_PROPS] = 84.84738207f;
	mapConst_k_2_0[MOTOR_BIGGER_PARROT_PROPS] = 0.65309704f;
	mapConst_k_2_1[MOTOR_BIGGER_PARROT_PROPS] = -0.13852527;
	mapConst_k_3_0[MOTOR_BIGGER_PARROT_PROPS] = -1.34462353e-04f;
	mapConst_k_3_1[MOTOR_BIGGER_PARROT_PROPS] = 3.57662798e-05f;
}

static void usage() {
	printf("usage:\n");
	printf("Normal operation: cf_motors {start|stop|status|setMotorType}\n");
	printf(
			"\tTest operation (replace <Ni> with desired speed for the four motors [rad/s]: cf_motors test N1 N2 N3 N4\n");
	printf(
			"\t\tif you provide only one, all motors spin at same speed: cf_motors test N\n");
	printf("\n");
	printf(
			"Set the motor type by calling `cf_motors setMotorType X`, with X an integer:\n");
	printf("\t%d->MOTOR_INVALID\n", int(MOTOR_INVALID));
	printf("\t%d->MOTOR_TEST_CMD_FEEDTHROUGH\n",
			int(MOTOR_TEST_CMD_FEEDTHROUGH));
	printf("\t%d->MOTOR_CF_STANDARD\n", int(MOTOR_CF_STANDARD));
	printf("\t%d->MOTOR_BIGGER_PARROT_PROPS\n", int(MOTOR_BIGGER_PARROT_PROPS));
	return;
}

int pwm_value_from_speed(const float speed, float battV) {
	float const MIN_BATT_V = 3.1f;
	float const MAX_BATT_V = 4.1f;

	if (battV < MIN_BATT_V) {
		battV = MIN_BATT_V;
	} else if (battV > MAX_BATT_V) {
		battV = MAX_BATT_V;
	}

	float const k_1 = mapConst_k_1_0[currentMotorType]
			+ mapConst_k_1_1[currentMotorType] * battV;
	float const k_2 = mapConst_k_2_0[currentMotorType]
			+ mapConst_k_2_1[currentMotorType] * battV;
	float const k_3 = mapConst_k_3_0[currentMotorType]
			+ mapConst_k_3_1[currentMotorType] * battV;

	int outpwm = int(k_1 + (k_2 + k_3 * speed) * speed);

	int const MIN_PWM = 0;
	int const MAX_PWM = 210;
	if (outpwm < MIN_PWM) {
		return MIN_PWM;
	}
	if (outpwm > MAX_PWM) {
		return MAX_PWM;
	}
	return outpwm;
}

int testMotors(float desiredSpeed[4]) {
	initConstants();
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
		uint64_t tElapsed_us = hrt_absolute_time() - t0; //[us]
		if (tElapsed_us >= (RUN_TIME * 1000000)) {
			break;
		}
		bool updated = false;
		orb_check(orb_sub_battery, &updated);
		if (updated) {
			numBattMeas++;
			orb_copy(ORB_ID(battery_status), orb_sub_battery, &batt);
			//update battery voltage estimate:
			batt_v = (1 - battFiltConst) * batt_v
					+ battFiltConst * batt.voltage_v;
		}

		if (tElapsed_us > nextPrintTime) {
			nextPrintTime += printPeriod_us;
			printf("batt_v = %.3f (over %d meas)\n", double(batt_v),
					int(numBattMeas));
		}

		for (int i = 0; i < 4; i++) {
			int ret = ioctl(fd, PWM_SERVO_SET(i),
					pwm_value_from_speed(desiredSpeed[i], batt_v));

			if (ret != OK) {
				err(1, "PWM_SERVO_SET(%d) = %d (%.3f)", i,
						pwm_value_from_speed(desiredSpeed[i], batt_v),
						double(desiredSpeed[i]));
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
	initConstants();
	orb_sub_actuator_direct = orb_subscribe(ORB_ID(actuator_direct));
	orb_sub_battery = orb_subscribe(ORB_ID(battery_status));

	if (orb_sub_actuator_direct <= 0) {
		printf("Error in orb_sub_actuator_direct  = %d\n",
				orb_sub_actuator_direct);
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
	const unsigned MOTOR_TIMEOUT_ms = 50; //if we don't receive a command every this often, kill motors.

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
				batt_v = (1 - battFiltConst) * batt_v
						+ battFiltConst * batt.voltage_v;
			}

			numCmdsReceived++;
			orb_copy(ORB_ID(actuator_direct), orb_sub_actuator_direct,
					&actuatorCmds);
			if (actuatorCmds.nvalues == 0 && motorsOff) {
				//empty command, most likely the first we get. Skip it.
				continue;
			}
			if (actuatorCmds.nvalues != 4) {
				//Incorrect number of motors!
				err(1, "PWM: incorrect number of motors! %d",
						actuatorCmds.nvalues);
				thread_running = false;
				break;
			}

			for (int i = 0; i < 4; i++) {
				if (actuatorCmds.values[i] > 0) {
					motorsOff = false;
				}

				int const pwmVal = pwm_value_from_speed(actuatorCmds.values[i],
						batt_v);
				int ret = ioctl(fd, PWM_SERVO_SET(i), pwmVal);

				if (ret != OK) {
					err(1, "PWM_SERVO_SET(%d) = %d (%.3f)", i, pwmVal,
							double(actuatorCmds.values[i]));
				}
			}
		}

		usleep(updatePeriod_us); //TODO THis isn't very nice, there will be jitter between logic creating motor commands, and them beign executed.
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
	initConstants();

	if (argc < 1) {
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "test")) {
		if (thread_running) {
			printf("A thread already running\n");
			return -1;
		}

		float desSpeed[4];
		if (argc == 3) {
			for (int i = 0; i < 4; i++) {
				desSpeed[i] = atof(argv[2]);
			}
		} else if (argc == 6) {
			for (int i = 0; i < 4; i++) {
				desSpeed[i] = atof(argv[2 + i]);
			}
		} else {
			usage();
			return -1;
		}

		testMotors(desSpeed);

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

		if (0 != param_get(param_find("MOTOR_TYPE"), &currentMotorType)) {
			printf("Failed to get param <MOTOR_TYPE>\n");
			currentMotorType = 0;
		}

		usleep(100000);

		thread_running = true;

		daemon_task = px4_task_spawn_cmd("cf_motors", SCHED_DEFAULT,
		SCHED_PRIORITY_DEFAULT + 5, 2000, cf_motors_thread,
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
		printf("\tCurrent motor type = %d\n", currentMotorType);
		printf("\tVoltage = %.3fV\n", double(batt_v));
		return 0;
	}

	usage();
	return -1;
}
