/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>

#include "DW1000Device.h"

#include "testFunctions.hpp"

#include "P2PRanging.h"

extern "C" __EXPORT int mtest_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static bool thread_should_print_status = false;
static bool thread_should_clearSysStatus = false;
static int daemon_task;

static uint8_t targetId = 0;

int mtest_thread_main(int argc, char *argv[]);

void usage();

void usage()
{
	printf("Need at least one argument:\n");
	printf("\tA -> start ranging Anchor (sink state)\n");
	printf("\tT -> start ranging Tag (sink state)\n");
//	printf("\tc -> basic connectivity test\n");
//	printf("\ts -> basic sender\n");
//	printf("\tr -> basic receiver\n");
//	printf("\tt -> Timing test\n");

}

int mtest_main(int argc, char *argv[])
{
	if (argc <= 1)
	{
		usage();
		return 0;
	}
	/*
	if (!strcmp(argv[1], "t"))
	{
		return markTestTimestampTest();
	}
	if (!strcmp(argv[1], "c"))
	{
		return basicConnectivityTest();
	}
	if (!strcmp(argv[1], "r"))
	{
		return basicReceiver();
	}
	if (!strcmp(argv[1], "s"))
	{
		return basicSender();
	}
	*/

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
            thread_should_print_status  = true;
		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "clear")) {
		if (thread_running) {
			thread_should_clearSysStatus = true;
		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "targetOff")) {
		if (thread_running) {
			targetId = 0;
		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "targetOn")) {
		if (thread_running) {
			targetId = 2;
		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("dw1000p2pranging",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 mtest_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}



	usage();
	return -1;
}

int mtest_thread_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "A") || !strcmp(argv[1], "T"))
	{
        printf("### P2P-ranging ###\n");
		static DW1000NS::P2PRanging p2pRanging;

		bool isRequester = false;

		uint8_t myId;
		if(!strcmp(argv[1], "T")){
			isRequester = true;
		}

		if(isRequester){
            myId = 1;
            targetId = 2;
		}
		else{
            myId = 2;
		}

		if (p2pRanging.Initialize(myId,10))
		{
			printf("Init failed, returning.");
			return -1;
		}

        thread_running = true;

		for (;;)
		{
            if(isRequester){
            	//always range to this guy (this will force doing an immediate range after completion)
                p2pRanging.setRangingTarget(targetId);
            }
			p2pRanging.runLoop();
			usleep(100);
			if(thread_should_exit)
			{
				break;
			}
			if(thread_should_clearSysStatus)
			{
                thread_should_clearSysStatus = false;
                p2pRanging.clearSysStatus();
			}
			if(thread_should_print_status)
			{
				thread_should_print_status = false;
				p2pRanging.printStatus();
			}
		}
        thread_running = false;
		return 0;
	}
	printf("Unrecognized command\n");
	return -1;
}
