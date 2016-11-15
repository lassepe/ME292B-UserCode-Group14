#pragma once
#include <stdint.h>

#include "DW1000Device.h"

class P2PRanging
{
public:
	enum MessageTypes
	{
		POLL = 0, //initiate the ranging exchange A->B.
		POLL_ACK = 1, //acknowledge receipt of "POLL", B->A
		RANGE = 2, //Third message in exchange, A->B
		RANGE_REPORT = 3, //Computed range, B->A
		RANGE_FAILED = 255, //exchange failed.
	};

	enum Status
	{
		LISTENING = 0, //when the unit isn't doing anything
		COMMUNICATING = 1, //when the unit is executing a ranging exchange
	};

	enum
	{ //some constants
		LEN_DATA = 16,
		DEFAULT_RESET_PERIOD_MS = 250,
		DEFAULT_DELAY_TIME_US = 3000,
	};

	P2PRanging();

	static int Initialize();

	static void LoopFunction(void);

	static Status getStatus()
	{
		return _status;
	}

	static void rangingTagLoop();
	static void rangingAnchorLoop();

private:
	static void noteActivity()
	{
		lastActivityTime_ms = DW1000Device::getTimeMillis();
	}

	static void resetInactive();

	static void handleSent();
	static void handleReceived();

	//the four message types
	static void transmitPoll();
	static void transmitPollAck();
	static void transmitRange();
	static void transmitRangeReport(const float curRange);
	static void transmitRangeFailed();

	static void computeRangeAsymmetric();
	static void computeRangeSymmetric();

	static void rangingReceiver();//->setUpAsReceiver();

	static volatile Status _status;

	static volatile MessageTypes expectedMsgId;

	// message sent/received state
	static volatile bool sentAck;
	static volatile bool receivedAck;
	// protocol error state
	static bool protocolFailed;
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
	static uint8_t data[LEN_DATA];
	// watchdog and reset period
	static uint32_t lastActivityTime_ms;
	static uint32_t resetPeriod_ms;
	// reply times (same on both sides for symm. ranging)
	static uint16_t replyDelayTime_us;
	// ranging counter (per second)
	static uint16_t successRangingCount;
	static uint32_t rangingCountPeriod;
	static float samplingRate;
};

