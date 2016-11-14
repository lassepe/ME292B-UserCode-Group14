#include "rangingExample.h"

#include <stdint.h>

#include "DW1000Ranging.h"
#include "DW1000Device.h"

// messages used in the ranging protocol
enum RangingProtocolMessages{
    POLL = 0,
    POLL_ACK = 1,
    RANGE = 2,
    RANGE_REPORT = 3,
    RANGE_FAILED = 255,
};

// message flow state
static volatile uint8_t expectedMsgId = POLL_ACK;
// message sent/received state
static volatile bool sentAck = false;
static volatile bool receivedAck = false;
// protocol error state
static bool protocolFailed = false;
// timestamps to remember
static DW1000Time timePollSent;
static DW1000Time timePollReceived;
static DW1000Time timePollAckSent;
static DW1000Time timePollAckReceived;
static DW1000Time timeRangeSent;
static DW1000Time timeRangeReceived;
// last computed range/time
static DW1000Time timeComputedRange;
// data buffer
#define LEN_DATA 16
static uint8_t data[LEN_DATA];
// watchdog and reset period
static uint32_t lastActivity;
static uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
static uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
static uint16_t successRangingCount = 0;
static uint32_t rangingCountPeriod = 0;
static float samplingRate = 0;

void noteActivity();
void rangingTagResetInactive();
void rangingHandleSent();
void rangingHandleReceived();
void rangingTagTransmitPoll();
void rangingTagTransmitRange();
void rangingAnchorResetInactive();
void rangingHandleSent();
void rangingHandleReceived();
void rangingAnchorTransmitPollAck();
void transmitRangeReport(float curRange);
void transmitRangeFailed();
void transmitPollAck();
void computeRangeAsymmetric();
void computeRangeSymmetric();
void rangingReceiver();

void rangingTagSetup() {
    // DEBUG monitoring
    printf("### DW1000-ranging-tag ###\n");
    // initialize the driver
    DW1000.begin();
    DW1000.configure();
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(2);
    DW1000.setNetworkId(10);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration();
    printf("Committed configuration ...\n");

    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(rangingHandleSent);
    DW1000.attachReceivedHandler(rangingHandleReceived);

	DW1000.interruptOnReceived(true);
	DW1000.interruptOnReceiveFailed(true);
	DW1000.interruptOnReceiveTimeout(true);
	DW1000.interruptOnSent(true);
	DW1000.writeSystemEventMaskRegister();
    // anchor starts by transmitting a POLL message
    rangingReceiver();
    rangingTagTransmitPoll();
    noteActivity();
}

void rangingAnchorSetup() {
    printf("### DW1000-ranging-tag ###\n");
    // initialize the driver
    DW1000.begin();
    DW1000.configure();
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(1);
    DW1000.setNetworkId(10);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration();
    printf("Committed configuration ...\n");

    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(rangingHandleSent);
    DW1000.attachReceivedHandler(rangingHandleReceived);

	DW1000.interruptOnReceived(true);
	DW1000.interruptOnReceiveFailed(true);
	DW1000.interruptOnReceiveTimeout(true);
	DW1000.interruptOnSent(true);
	DW1000.writeSystemEventMaskRegister();
    // anchor starts in receiving mode, awaiting a ranging poll message
    rangingReceiver();
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = DW1000Device::getTimeMillis();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = DW1000Device::getTimeMillis();
}

void rangingTagResetInactive() {
    // tag sends POLL and listens for POLL_ACK
    expectedMsgId = POLL_ACK;
    rangingTagTransmitPoll();
    noteActivity();
}

void rangingAnchorResetInactive() {
    // anchor listens for POLL
    expectedMsgId = POLL;
    rangingReceiver();
    noteActivity();
}

void rangingHandleSent() {
    // status change on sent success
    sentAck = true;
}

void rangingHandleReceived() {
    // status change on received success
    receivedAck = true;
}

void rangingAnchorTransmitPollAck() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL_ACK;
    // delay the same amount as ranging tag
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeReport(float curRange) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeFailed() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_FAILED;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}


void transmitPollAck() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL_ACK;
    // delay the same amount as ranging tag
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void rangingTagTransmitPoll() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void rangingTagTransmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    // delay sending the message and remember expected future sent timestamp
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void rangingReceiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

/*
 * RANGING ALGORITHMS
 * ------------------
 * Either of the below functions can be used for range computation (see line "CHOSEN
 * RANGING ALGORITHM" in the code).
 * - Asymmetric is more computation intense but least error prone
 * - Symmetric is less computation intense but more error prone to clock drifts
 *
 * The anchors and tags of this reference example use the same reply delay times, hence
 * are capable of symmetric ranging (and of asymmetric ranging anyway).
 */

void computeRangeAsymmetric() {
    // asymmetric two-way ranging (more computation intense, less error prone)
    DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
    DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
    DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
    DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

void computeRangeSymmetric() {
    // symmetric two-way ranging (less computation intense, more error prone on clock drift)
    DW1000Time tof = ((timePollAckReceived - timePollSent) - (timePollAckSent - timePollReceived) +
                      (timeRangeReceived - timePollAckSent) - (timeRangeSent - timePollAckReceived)) * 0.25f;
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */


void rangingTagLoop() {
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (DW1000Device::getTimeMillis() - lastActivity > resetPeriod) {
            rangingTagResetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        uint8_t msgId = data[0];
        if (msgId == POLL) {
            DW1000.getTransmitTimestamp(timePollSent);
            //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
        } else if (msgId == RANGE) {
            DW1000.getTransmitTimestamp(timeRangeSent);
            noteActivity();
        }
    }
    if (receivedAck) {
        receivedAck = false;
        // get message and parse
        DW1000.getData(data, LEN_DATA);
        uint8_t msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again
            //Serial.print("Received wrong message # "); Serial.println(msgId);
            expectedMsgId = POLL_ACK;
            rangingTagTransmitPoll();
            return;
        }
        if (msgId == POLL_ACK) {
            DW1000.getReceiveTimestamp(timePollAckReceived);
            expectedMsgId = RANGE_REPORT;
            rangingTagTransmitRange();
            noteActivity();
        } else if (msgId == RANGE_REPORT) {
            expectedMsgId = POLL_ACK;
            float curRange;
            memcpy(&curRange, data + 1, 4);
            rangingTagTransmitPoll();
            noteActivity();
        } else if (msgId == RANGE_FAILED) {
            expectedMsgId = POLL_ACK;
            rangingTagTransmitPoll();
            noteActivity();
        }
    }
}

void rangingAnchorLoop() {
    int32_t curMillis = DW1000Device::getTimeMillis();
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
        	rangingAnchorResetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        uint8_t msgId = data[0];
        if (msgId == POLL_ACK) {
            DW1000.getTransmitTimestamp(timePollAckSent);
            noteActivity();
        }
    }
    if (receivedAck) {
        receivedAck = false;
        // get message and parse
        DW1000.getData(data, LEN_DATA);
        uint8_t msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
        }
        if (msgId == POLL) {
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;
            DW1000.getReceiveTimestamp(timePollReceived);
            expectedMsgId = RANGE;
            transmitPollAck();
            noteActivity();
        }
        else if (msgId == RANGE) {
            DW1000.getReceiveTimestamp(timeRangeReceived);
            expectedMsgId = POLL;
            if (!protocolFailed) {
                timePollSent.setTimestamp(data + 1);
                timePollAckReceived.setTimestamp(data + 6);
                timeRangeSent.setTimestamp(data + 11);
                // (re-)compute range as two-way ranging is done
                computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
                transmitRangeReport(timeComputedRange.getAsMicroSeconds());
                float distance = timeComputedRange.getAsMeters();
                printf("Range: %dmm\n", int(1000.f*distance+0.5f));
                printf("\t RX power: %f dDm\n", double(DW1000.getReceivePower()));
                printf("\t Sampling: %d.%03d Hz\n", int(0.5f+samplingRate), int(0.5f+1000*samplingRate)%1000);
                //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
                //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
                //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
                // update sampling rate (each second)
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            }
            else {
                transmitRangeFailed();
            }

            noteActivity();
        }
    }
}


