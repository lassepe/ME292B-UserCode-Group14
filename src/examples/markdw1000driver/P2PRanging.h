#pragma once
#include <stdint.h>

#include "DW1000Device.h"

class P2PRanging
{
public:
	enum MessageTypes
	{
		MSG_RANGING_INIT = 0, //initiate the ranging exchange A->B.
		MSG_RANGING_REPLY1 = 1, //acknowledge receipt of "MSG_RANGING_INIT", B->A
		MSG_RANGING_REPLY2 = 2, //Third message in exchange, A->B
		MSG_RANGING_REPORT = 3, //Computed range, B->A
		MSG_RANGING_FAILED = 255, //exchange failed.
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


	void setAutoTransmitRangingInit(bool in)
	{
		_autoTxRangingInit = in;
	}

	static void runLoop();

private:
	static void noteActivity()
	{
		_lastActivityTime_ms = DW1000Device::getTimeMillis();
	}

	static void resetInactive();

	static void handleSent();
	static void handleReceived();

	//the four message types
	static void transmitRangingInit();
	static void transmitRangingReply1();
	static void transmitRange();
	static void transmitRangeReport(const float curRange);
	static void transmitRangeFailed();

	static void computeRangeAsymmetric();
	static void computeRangeSymmetric();

	static void rangingReceiver(); //->setUpAsReceiver();

	static volatile MessageTypes _expectedMsgId;

	// message sent/received state
	static volatile bool _sentAck;
	static volatile bool _receivedAck;
	// protocol error state
	static bool _protocolFailed;
	// timestamps to remember
	static DW1000Time _timeRangingInitSent;
	static DW1000Time _timeRangingInitReceived;
	static DW1000Time _timeRangingReply1Sent;
	static DW1000Time _timeRangingReply1Received;
	static DW1000Time _timeRangeReportSent;
	static DW1000Time _timeRangeReportReceived;
	// last computed range/time
	static DW1000Time _timeComputedRange;
	// data buffer
	static uint8_t _data[LEN_DATA];
	// watchdog and reset period
	static uint32_t _lastActivityTime_ms;
	static uint32_t _resetPeriod_ms;
	// reply times (same on both sides for symm. ranging)
	static uint16_t _replyDelayTime_us;
	// ranging counter (per second)
	static uint16_t _successRangingCount;
	static uint32_t _rangingCountPeriod;
	static float _samplingRate;
	static bool _autoTxRangingInit;
};

