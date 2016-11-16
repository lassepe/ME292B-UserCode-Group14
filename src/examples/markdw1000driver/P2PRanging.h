#pragma once
#include <stdint.h>

#include "DW1000.h"
#include "DW1000Device.h"
#include "DW1000Constants.h"

namespace DW1000NS{
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

	enum MessageFields
	{
		MSG_FIELD_TYPE = 0, //what type of message is this?
		MSG_FIELD_REQUESTER_ID = MSG_FIELD_TYPE + 1,
		MSG_FIELD_RESPONDER_ID = MSG_FIELD_REQUESTER_ID + 1,
		MSG_FIELD_INTERACTION_COUNTER = MSG_FIELD_RESPONDER_ID + 1,
		MSG_FIELD_DATA_START = MSG_FIELD_INTERACTION_COUNTER + 1,
	};

	enum
	{ //some constants
		LEN_DATA = 19,
		DEFAULT_RESET_PERIOD_MS = 250,
		DEFAULT_DELAY_TIME_US = 3000,
	};

	P2PRanging();

	static int Initialize(uint8_t deviceId, uint16_t networkId);

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
	static void transmitRangingReply2();
	static void transmitRangeReport(const uint32_t curRange_um);
	static void transmitRangeFailed();

	static void computeRangeAsymmetric();
	static void computeRangeSymmetric();

	static void rangingReceiver(); //->setUpAsReceiver();

	static volatile MessageTypes _expectedMsg;
	static volatile MessageTypes _lastTxMsg;
//	static volatile MessageTypes _lastRxMsg;

	// message sent/received state
	static volatile bool _haveUnhandledSentMsg;
	static volatile bool _haveUnhandledReceivedMsg;
	// protocol error state
	static volatile bool _protocolFailed;

	// timestamps to remember
//	static volatile uint8_t _timestampRaw_RangingInit_tx[LEN_RX_STAMP];
//	static volatile uint8_t _timestampRaw_RangingInit_rx[LEN_RX_STAMP];
//	static volatile uint8_t _timestampRaw_Reply1_tx[LEN_RX_STAMP];
//	static volatile uint8_t _timestampRaw_Reply1_rx[LEN_RX_STAMP];
//	static volatile uint8_t _timestampRaw_Reply2_tx[LEN_RX_STAMP];
//	static volatile uint8_t _timestampRaw_Reply2_rx[LEN_RX_STAMP];

	// timestamps to remember
	static DW1000Time _timeRangingInitSent;
	static DW1000Time _timeRangingInitReceived;
	static DW1000Time _timeRangingReply1Sent;
	static DW1000Time _timeRangingReply1Received;
	static DW1000Time _timeRangingReply2Sent;
	static DW1000Time _timeRangingReply2Received;
	// last computed range/time
	static DW1000Time _timeComputedRange;
	// data buffer
	static uint8_t _rxData[LEN_DATA];
	static uint8_t _txData[LEN_DATA];
	// watchdog and reset period
	static volatile uint32_t _lastActivityTime_ms;
	static uint32_t _resetPeriod_ms;
	// reply times (same on both sides for symm. ranging)
	static uint16_t _replyDelayTime_us;
	// ranging counter (per second)
	static uint16_t _successRangingCount;
	static uint32_t _rangingCountPeriod;
	static float _samplingRate;
	static bool _autoTxRangingInit;

	static uint8_t _myId;
	static uint8_t _commPartnerId;//the Id of the other party we're talking to

};

}// namespace DW1000NS
