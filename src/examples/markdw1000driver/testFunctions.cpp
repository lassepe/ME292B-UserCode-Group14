#include "testFunctions.hpp"

using namespace DW1000NS;

// DEBUG packet sent status and count
static volatile bool sentAck = false;
static volatile unsigned long delaySent = 0;
static int16_t sentNum = 0; // todo check int type
static DW1000Time sentTime;

static volatile bool received = false;
static volatile bool error = false;
static volatile int16_t numReceived = 0; // todo check int type
#define MESSAGE_LEN 10
static uint8_t message[MESSAGE_LEN];

void printDeviceID();
void handleSent();
void transmitter();
void handleReceived();
void handleError();
void receiver();

int basicConnectivityTest(void){
	// initialize the driver
	DW1000.begin();
	if(DW1000.configure()) return -1;
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
	float temp, vbat;
	DW1000.getTempAndVbat(temp, vbat);
	printf("Device temp = %dC, vbat = %dmV\n", int(temp+0.5f), int(1000*vbat+0.5f));
	printf("====================================\n");

	printf("Running LED test\n");
	DW1000.enableAllLeds();
	return 0;
}

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
  DW1000.setTxData(msg,5);

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
	if(DW1000.configure()) return -1;

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
	if(DW1000.configure()) return -1;

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
			DW1000.getRxData(message, MESSAGE_LEN);
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
			DW1000.getRxData(message, MESSAGE_LEN);
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
	if(DW1000.configure()) return -1;
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
