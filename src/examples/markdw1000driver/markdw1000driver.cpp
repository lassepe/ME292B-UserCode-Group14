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

extern "C" __EXPORT int mtest_main(int argc, char *argv[]);

int basicConnectivityTest(void);
int basicSender(void);
void handleSent();
void transmitter();
int markTestTimestampTest(void);
void usage();
void printDeviceID();

int basicConnectivityTest(void){
	// initialize the driver
	DW1000.begin();
	DW1000.configure();
	printf("DW1000 initialized ... \n");
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDeviceAddress(5);
	DW1000.setNetworkId(10);
	DW1000.commitConfiguration();
	printf("Committed configuration ... \n");
	usleep(1000);

	printf("====================================\n");
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    printf("Device ID: %s\n",msg);
    DW1000.getPrintableExtendedUniqueIdentifier(msg);
    printf("Unique ID: %s\n",msg);
    DW1000.getPrintableNetworkIdAndShortAddress(msg);
    printf("Network ID & Device Address: %s\n",msg);
    DW1000.getPrintableDeviceMode(msg);
    printf("Device mode: %s\n",msg);
	printf("====================================\n");

	printf("Running LED test\n");
	DW1000.enableAllLeds();
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
  printf("Transmitting packet ... #%d\n",int(sentNum));
  DW1000.idle();
  DW1000.newTransmit();
  DW1000.setDefaults();
  uint8_t msg[5] = "1234";
  DW1000.setData(msg,5);

  // delay sending the message for the given amount
  DW1000Time deltaTime = DW1000Time(10, DW1000Time::MILLISECONDS);
  DW1000.setDelay(deltaTime);

  DW1000.startTransmit();
  delaySent = DW1000Device::getTimeMillis();
}

void printDeviceID(){
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    printf("Device ID: %s\n",msg);
}

int basicSender(void){
	printf("DW1000 sender test\n");
	// initialize the driver
	DW1000.begin();
    printDeviceID();
	DW1000.configure();

	// general configuration
	DW1000.newConfiguration();
	DW1000.setDeviceAddress(5);
	DW1000.setNetworkId(10);
	DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
	DW1000.commitConfiguration();

	// attach callback for (successfully) sent messages
	DW1000.interruptOnSent(true);
	DW1000.writeSystemEventMaskRegister();

	DW1000.attachSentHandler(handleSent);

    sentAck = false;
    transmitter();

	printf("Starting loop\n");
	unsigned failCount = 0;
	unsigned numOK = 0;
	for (unsigned i = 0; i < 2000; i++) {
		if (!sentAck) {
			failCount++;
			if(failCount > 500){
				printf("Failed! Resending ...\n");
				failCount = 0;
				transmitter();
			}
            usleep(1000);
			continue;
		}
		// continue on success confirmation
		sentAck = false;
		// update and print some information about the sent message
		printf("   delay sent [ms] = %d\n", int(DW1000Device::getTimeMillis() - delaySent));
		DW1000Time newSentTime;
		DW1000.getTransmitTimestamp(newSentTime);
		printf("Processed packet ... #%d\n",sentNum);
		printf("Sent timestamp ... %lf\n",double(newSentTime.getAsMicroSeconds()));
		// note: delta is just for simple demo as not correct on system time counter wrap-around
		printf("DW1000 delta send time [ms] ... %lf\n", double((newSentTime.getAsMicroSeconds() - sentTime.getAsMicroSeconds()) * 1.0e-3f));
		sentTime = newSentTime;
		sentNum++;
		// again, transmit some data
		transmitter();
		numOK ++;
		if(numOK > 10) break;
	}
	DW1000.idle();
	return 0;
}

volatile bool received = false;
volatile bool error = false;
volatile int16_t numReceived = 0; // todo check int type
#define MESSAGE_LEN 10
uint8_t message[MESSAGE_LEN];

void handleReceived();
void handleError();
void receiver();
int basicReceiver(void);

void handleReceived() {
  // status change on reception success
  received = true;
}

void handleError() {
  error = true;
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

int basicReceiver(void){
	printf("DW1000 receiver test\n");
	// initialize the driver
	DW1000.begin();
    printDeviceID();
	DW1000.configure();

	// general configuration
	DW1000.newConfiguration();
	DW1000.setDeviceAddress(5);
	DW1000.setNetworkId(10);
	DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
	DW1000.commitConfiguration();

	// attach callback for (successfully) sent messages
	DW1000.interruptOnReceived(true);
	DW1000.interruptOnReceiveFailed(true);
	DW1000.interruptOnReceiveTimeout(true);
	DW1000.writeSystemEventMaskRegister();

    DW1000.attachReceivedHandler(handleReceived);
    DW1000.attachReceiveFailedHandler(handleError);
    DW1000.attachErrorHandler(handleError);

    receiver();

	printf("Starting loop\n");
	for (unsigned i = 0; i < 10000; i++) {
		if (received) {
			numReceived++;
			// get data as string
			DW1000.getData(message, MESSAGE_LEN);
			printf("Received message ... #%d\n", int(numReceived));
			printf("Data is ... <%s>\n", message);
			printf("FP power is %f[dBm]\n", double(DW1000.getFirstPathPower()));
			printf("RX power is %f[dBm]\n", double(DW1000.getReceivePower()));
			printf("Signal quality is %f\n", double(DW1000.getReceiveQuality()));
			received = false;
		}
		if (error) {
			printf("Error receiving a message\n");
			error = false;
			DW1000.getData(message, MESSAGE_LEN);
			printf("Error data is <%s>\n", message);
		}
		usleep(1000);
	}
	DW1000.idle();
	return 0;
}

int markTestTimestampTest(void){
	printf("DW1000 timestamp test\n");
	// initialize the driver
	DW1000.begin();
	DW1000.configure();
	printf("DW1000 initialized ... \n");
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDeviceAddress(5);
	DW1000.setNetworkId(10);
	DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
	DW1000.commitConfiguration();
	printf("Committed configuration ... \n");

	printf("Starting loop\n");
    usleep(10000);
	for (unsigned i = 0; i < 20; i++) {
		DW1000Time t;
		DW1000.getSystemTimestamp(t);
		printf("i=%d, t = %lfms\n", i, double(t.getAsMicroSeconds()/1e3f));
		usleep(100*1000);
	}
	return 0;
}

void usage(){
		printf("Need at least one argument:\n");
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

	usage();
	return -1;
}
