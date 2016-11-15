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
		printf("\tA -> start ranging Anchor polished (sink state)\n");
		printf("\tT -> start ranging Tag polished (sink state)\n");
		printf("\t1 -> start ranging Anchor (sink state)\n");
		printf("\t2 -> start ranging Tag (sink state)\n");

		printf("\tc -> basic connectivity test\n");
		printf("\ts -> basic sender\n");
		printf("\tr -> basic receiver\n");
		printf("\tt -> Timing test\n");


}

void newRange();
void newDevice(DW1000Device* device);
void inactiveDevice(DW1000Device* device);
void newBlink(DW1000Device* device);

void newRange() {
  printf("from: %d\n", int(DW1000Ranging.getDistantDevice()->getShortAddress()));
  printf("\t Range: %dmm\n", int(1000*DW1000Ranging.getDistantDevice()->getRange()));
  printf("\t RX power: %f dBm\n", double(DW1000Ranging.getDistantDevice()->getRXPower()));
}

void newDevice(DW1000Device* device) {
  printf("ranging init; 1 device added ! -> ");
  printf(" short: %d\n", int(device->getShortAddress()));
}

void inactiveDevice(DW1000Device* device) {
  printf("delete inactive device: %d\n", int(device->getShortAddress()));
}

void newBlink(DW1000Device* device) {
  printf("blink; 1 device added ! -> ");
  printf(" short: %d\n", int(device->getShortAddress()));
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
		DW1000Ranging.initCommunication();

		DW1000Ranging.attachNewRange(newRange);
		DW1000Ranging.attachBlinkDevice(newBlink);
		DW1000Ranging.attachInactiveDevice(inactiveDevice);
		DW1000Ranging.useRangeFilter(false);

		DW1000Ranging.startAsAnchor("82:17:5B:D5:A9:9A:E2:9C",
				DW1000.MODE_LONGDATA_RANGE_ACCURACY);

        printf("Starting loop...\n");
        for(;;){
        	DW1000Ranging.loop();
        	usleep(1000);//approx. 1ms
        }
        return 0;//never get here...
	}

	if(!strcmp(argv[1], "T")){
		DW1000Ranging.initCommunication();

		DW1000Ranging.attachNewRange(newRange);
		DW1000Ranging.attachNewDevice(newDevice);
		DW1000Ranging.attachInactiveDevice(inactiveDevice);
		DW1000Ranging.useRangeFilter(false);

		DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C",
				DW1000.MODE_LONGDATA_RANGE_ACCURACY);

        printf("Starting loop...\n");
        for(;;){
        	DW1000Ranging.loop();
        	usleep(1000);//approx. 1ms
        }
        return 0;//never get here...
	}

	if(!strcmp(argv[1], "1")){
        rangingAnchorSetup();
        printf("Starting loop...\n");
        for(;;){
        	rangingAnchorLoop();
        	usleep(1000);//approx. 1ms
        }
        return 0;
	}

	if(!strcmp(argv[1], "2")){
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
