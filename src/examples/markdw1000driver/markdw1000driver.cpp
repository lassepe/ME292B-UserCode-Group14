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

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include "DW1000Ranging.h"
#include "DW1000Device.h"

#include "testFunctions.hpp"
#include "rangingExample.h"

extern "C" __EXPORT int mtest_main(int argc, char *argv[]);

void usage();

void usage(){
		printf("Need at least one argument:\n");
		printf("\tA -> start ranging Anchor (sink state)");
		printf("\tT -> start ranging Tag (sink state)\n");

		printf("\tc -> basic connectivity test\n");
		printf("\ts -> basic sender\n");
		printf("\tr -> basic receiver\n");
		printf("\tt -> Timing test\n");


}

int mtest_main(int argc, char *argv[]) {
	if(argc <= 1){
		usage();
		return 0;
	}
	if(!strcmp(argv[1], "t")){
        return markTestTimestampTest();
	}
	if(!strcmp(argv[1], "c")){
        return basicConnectivityTest();
	}
	if(!strcmp(argv[1], "r")){
        return basicReceiver();
	}
	if(!strcmp(argv[1], "s")){
        return basicSender();
	}

	if(!strcmp(argv[1], "A")){
        rangingAnchorSetup();
        printf("Starting loop...\n");
        for(;;){
        	rangingAnchorLoop();
        	usleep(1000);//approx. 1ms
        }
        return 0;
	}

	if(!strcmp(argv[1], "T")){
        rangingTagSetup();
        printf("Starting loop...\n");
        for(;;){
        	rangingTagLoop();
        	usleep(1000);//approx. 1ms
        }
        return 0;
	}

	usage();
	return -1;
}
