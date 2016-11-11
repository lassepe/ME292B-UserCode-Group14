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
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include "DW1000Ranging.h"

extern "C" __EXPORT int mtest_main(int argc, char *argv[]);

int basicConnectivityTest(void);
int basicSender(void);
void handleSent();
void transmitter();

int basicConnectivityTest(void){
	// initialize the driver
	DW1000.begin();
	DW1000.select(0);
	printf("DW1000 initialized ... \n");
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDeviceAddress(5);
	DW1000.setNetworkId(10);
	DW1000.commitConfiguration();
	printf("Committed configuration ... \n");
	// wait a bit
	usleep(1000*1000);

	for (unsigned i = 0; i < 2; i++) {
		// DEBUG chip info and registers pretty printed
		char msg[128];
		DW1000.getPrintableDeviceIdentifier(msg);
		printf("Device ID: ");
		printf(msg);
		printf("\n");
		DW1000.getPrintableExtendedUniqueIdentifier(msg);
		printf("Unique ID: ");
		printf(msg);
		printf("\n");
		DW1000.getPrintableNetworkIdAndShortAddress(msg);
		printf("Network ID & Device Address: ");
		printf(msg);
		printf("\n");
		DW1000.getPrintableDeviceMode(msg);
		printf("Device mode: ");
		printf(msg);
		printf("\n");
		// wait a bit
        usleep(1000*1000);
	}
	return 0;
}

// DEBUG packet sent status and count
bool sent = false;
volatile bool sentAck = false;
volatile unsigned long delaySent = 0;
int16_t sentNum = 0; // todo check int type
DW1000Time sentTime;

void handleSent() {
  // status change on sent success
  sentAck = true;
}

void transmitter() {
  // transmit some data
  printf("Transmitting packet ... #\n");
  printf("%d\n",int(sentNum));
  DW1000.newTransmit();
  DW1000.setDefaults();
  uint8_t msg[5];
  msg[0] = '1';
  msg[1] = '2';
  msg[2] = '3';
  msg[3] = '4';
  msg[4] = 0;

  DW1000.setData(msg,5);
  // delay sending the message for the given amount
  DW1000Time deltaTime = DW1000Time(10, DW1000Time::MILLISECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.startTransmit();
  delaySent = DW1000Device::getTimeMillis();
}

int basicSender(void){
	printf("DW1000 sender test\n");
	// initialize the driver
	DW1000.begin();
	DW1000.select(0);
	printf("DW1000 initialized ... \n");
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDeviceAddress(5);
	DW1000.setNetworkId(10);
	DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
	DW1000.commitConfiguration();
	printf("Committed configuration ... \n");

	char msg[128];
	DW1000.getPrintableDeviceIdentifier(msg);
	printf("Device ID: ");
	printf("\n");
	printf(msg);
	printf("\n");
	DW1000.getPrintableExtendedUniqueIdentifier(msg);
	printf("Unique ID: ");
	printf("\n");
	printf(msg);
	printf("\n");
	DW1000.getPrintableNetworkIdAndShortAddress(msg);
	printf("Network ID & Device Address: ");
	printf("\n");
	printf(msg);
	printf("\n");
	DW1000.getPrintableDeviceMode(msg);
	printf("Device mode: ");
	printf("\n");
	printf(msg);
	printf("\n");
	// attach callback for (successfully) sent messages
	DW1000.attachSentHandler(handleSent);

    sentAck = true;

	printf("Starting loop\n");
    usleep(10000);
	for (unsigned i = 0; i < 1000; i++) {
		if (!sentAck) {
            usleep(1000);
			continue;
		}
		// continue on success confirmation
		// (we are here after the given amount of send delay time has passed)
		sentAck = false;
		// update and print some information about the sent message
		printf("ARDUINO delay sent [ms] ... \n");
		printf("%d\n", int(DW1000Device::getTimeMillis() - delaySent));
        usleep(10000);
		DW1000Time newSentTime;
		DW1000.getTransmitTimestamp(newSentTime);
		printf("Processed packet ... #"); printf("\n");
		printf("%d\n",sentNum);
		printf("Sent timestamp ... \n");
		printf("%lf\n",double(newSentTime.getAsMicroSeconds()));
        usleep(10000);
		// note: delta is just for simple demo as not correct on system time counter wrap-around
		printf("DW1000 delta send time [ms] ... \n");
		printf("%lf\n", double((newSentTime.getAsMicroSeconds() - sentTime.getAsMicroSeconds()) * 1.0e-3f));
        usleep(10000);
		sentTime = newSentTime;
		sentNum++;
		// again, transmit some data
		transmitter();
	}
	return 0;
}


int mtest_main(int argc, char *argv[]) {
	return basicSender();
}
