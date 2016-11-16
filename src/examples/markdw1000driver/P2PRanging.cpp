#include "P2PRanging.h"
#include <systemlib/perf_counter.h>

#include "DW1000.h"

using namespace DW1000NS;

//definitions of static members:
volatile P2PRanging::MessageTypes P2PRanging::_expectedMsg;
volatile P2PRanging::MessageTypes P2PRanging::_lastTxMsg;

volatile bool P2PRanging::_haveUnhandledSentMsg;
volatile bool P2PRanging::_haveUnhandledReceivedMsg;
volatile bool P2PRanging::_protocolFailed;
DW1000Time P2PRanging::_timeRangingInitSent;
DW1000Time P2PRanging::_timeRangingInitReceived;
DW1000Time P2PRanging::_timeRangingReply1Sent;
DW1000Time P2PRanging::_timeRangingReply1Received;
DW1000Time P2PRanging::_timeRangingReply2Sent;
DW1000Time P2PRanging::_timeRangingReply2Received;
DW1000Time P2PRanging::_timeComputedRange;
uint8_t P2PRanging::_rxData[P2PRanging::LEN_DATA];
uint8_t P2PRanging::_txData[P2PRanging::LEN_DATA];
volatile uint32_t P2PRanging::_lastActivityTime_ms;
uint32_t P2PRanging::_resetPeriod_ms;
uint16_t P2PRanging::_replyDelayTime_us;
uint16_t P2PRanging::_successRangingCount;
uint32_t P2PRanging::_rangingCountPeriod;
float P2PRanging::_samplingRate;
bool P2PRanging::_autoTxRangingInit;
uint8_t P2PRanging::_myId;
uint8_t P2PRanging::_commPartnerId;

perf_counter_t pc_TxCount(perf_alloc(PC_COUNT, "dw1000_txCount"));
perf_counter_t pc_RxCount(perf_alloc(PC_COUNT, "dw1000_rxCount"));
perf_counter_t pc_timeout(perf_alloc(PC_COUNT, "dw1000_timeout"));
perf_counter_t pc_count_RangingFailed(perf_alloc(PC_COUNT, "dw1000_rangingFailed"));
perf_counter_t pc_count_RangingSucceeded(perf_alloc(PC_COUNT, "dw1000_rangingSucceeded"));
perf_counter_t pc_count_unexpectedMsg(perf_alloc(PC_COUNT, "dw1000_unexpectedMsg"));

perf_counter_t pc_time_wholeLoop(perf_alloc(PC_ELAPSED, "dw1000_spiReadData"));
perf_counter_t pc_time_loopRxSec(perf_alloc(PC_ELAPSED, "dw1000_loopRx"));
perf_counter_t pc_time_loopTxSec(perf_alloc(PC_ELAPSED, "dw1000_loopTx"));
perf_counter_t pc_time_readData(perf_alloc(PC_ELAPSED, "dw1000_spiReadData"));

//requester:
perf_counter_t	pc_time_SInit_RReply1(perf_alloc(PC_ELAPSED, "dw1000_timeSendInitReceiveReply1"));
perf_counter_t	pc_time_RReply1_SReply2(perf_alloc(PC_ELAPSED, "dw1000_timeReceiveReply1SendReply2"));
perf_counter_t	pc_time_SReply2_RRange(perf_alloc(PC_ELAPSED, "dw1000_timeSendReply2ReceiveRangeRept"));
perf_counter_t	pc_time_RRange_SInit(perf_alloc(PC_ELAPSED, "dw1000_timeReceiveRangeSendInit"));
perf_counter_t	pc_time_LoopRequester(perf_alloc(PC_ELAPSED, "dw1000_rangingLoopTimeRequester"));

//responder
perf_counter_t	pc_time_RInit_SReply1(perf_alloc(PC_ELAPSED, "dw1000_timeReceiveInitSendReply1"));
perf_counter_t	pc_time_SReply1_RReply2(perf_alloc(PC_ELAPSED, "dw1000_timeSendReply1ReceiveReply2"));
perf_counter_t	pc_time_RReply2_SRange(perf_alloc(PC_ELAPSED, "dw1000_timeReceiveReply2SendRange"));
perf_counter_t	pc_time_SRange_RInit(perf_alloc(PC_ELAPSED, "dw1000_timeSendRangeReceiveInit"));
perf_counter_t	pc_time_LoopResponder(perf_alloc(PC_ELAPSED, "dw1000_rangingLoopTimeResponder"));



P2PRanging::P2PRanging()
{
	// message sent/received state
	_haveUnhandledSentMsg = false;
	_haveUnhandledReceivedMsg = false;
	_protocolFailed = false;

	_resetPeriod_ms = DEFAULT_RESET_PERIOD_MS;
	// reply times (same on both sides for symm. ranging)
	_replyDelayTime_us = DEFAULT_DELAY_TIME_US;
	// ranging counter (per second)
	_successRangingCount = 0;
	_rangingCountPeriod = 0;
	_samplingRate = 0;
	_lastActivityTime_ms = 0;

	_autoTxRangingInit = false;
}

int P2PRanging::Initialize(uint8_t deviceId, uint16_t networkId)
{
	// initialize the driver
	DW1000.begin();
	if (DW1000.configure())
		return -1;
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	_myId = deviceId;
	DW1000.setDeviceAddress(uint16_t(_myId));//mwm TODO magic number
	DW1000.setNetworkId(networkId);//mwm TODO magic number
	DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY); //somewhat arbitrary mode
	DW1000.commitConfiguration();

	DW1000.enableAllLeds();

	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(P2PRanging::handleSent);
	DW1000.attachReceivedHandler(P2PRanging::handleReceived);

	DW1000.interruptOnReceived(true);
	DW1000.interruptOnReceiveFailed(true);
	DW1000.interruptOnReceiveTimeout(true);
	DW1000.interruptOnSent(true);
	DW1000.writeSystemEventMaskRegister();
	resetInactive();

	return 0;
}

void P2PRanging::resetInactive()
{
	_expectedMsg = MSG_RANGING_INIT; //listen for this, if we don't know what else is going on.
	noteActivity();
	rangingReceiver();
}

void P2PRanging::handleSent()
{
	// status change on sent success
	_haveUnhandledSentMsg = true;
	perf_count(pc_TxCount);
}

void P2PRanging::handleReceived()
{
	// status change on received success
	_haveUnhandledReceivedMsg = true;
	perf_count(pc_RxCount);
	return;
}

void P2PRanging::transmitRangingInit()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_INIT;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_REQUESTER_ID] = _myId;
	_txData[MSG_FIELD_RESPONDER_ID] = _commPartnerId;
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsg = MSG_RANGING_REPLY1;
}

void P2PRanging::transmitRangingReply1()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_REPLY1;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_REQUESTER_ID] = _commPartnerId;
	_txData[MSG_FIELD_RESPONDER_ID] = _myId;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTime_us, DW1000Time::MICROSECONDS);
	DW1000.setDelay(deltaTime);
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsg = MSG_RANGING_REPLY2;
}

void P2PRanging::transmitRangingReply2()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_REPLY2;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_REQUESTER_ID] = _myId;
	_txData[MSG_FIELD_RESPONDER_ID] = _commPartnerId;
	// delay sending the message and remember expected future sent timestamp
	DW1000Time deltaTime = DW1000Time(_replyDelayTime_us, DW1000Time::MICROSECONDS);
	_timeRangingReply2Sent = DW1000.setDelay(deltaTime);
	_timeRangingInitSent.getTimestamp(&_txData[MSG_FIELD_DATA_START]);
	_timeRangingReply1Received.getTimestamp(&_txData[MSG_FIELD_DATA_START] + 5);
	_timeRangingReply2Sent.getTimestamp(&_txData[MSG_FIELD_DATA_START] + 10);
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsg = MSG_RANGING_REPORT;
}

void P2PRanging::transmitRangeReport(const uint32_t curRange_um)
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_REPORT ;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_REQUESTER_ID] = _commPartnerId;
	_txData[MSG_FIELD_RESPONDER_ID] = _myId;
	// write final ranging result
    memcpy(&_txData[MSG_FIELD_DATA_START], &curRange_um, 4);
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsg = MSG_RANGING_INIT;
    //NOTE: cannot reset here, would cancel transmission resetInactive();
}

void P2PRanging::transmitRangeFailed()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_FAILED;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_REQUESTER_ID] = _commPartnerId;
	_txData[MSG_FIELD_RESPONDER_ID] = _myId;
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
    //NOTE: cannot reset here, would cancel transmission resetInactive();
}

void P2PRanging::rangingReceiver()
{
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}

void print_message_str(P2PRanging::MessageTypes msg);
void print_message_str(P2PRanging::MessageTypes msg){
	switch(msg){
	case P2PRanging::MSG_RANGING_INIT:
		printf("MSG_RANGING_INIT");
		return;
	case P2PRanging::MSG_RANGING_REPLY1:
		printf("MSG_RANGING_REPLY1");
		return;

	case P2PRanging::MSG_RANGING_REPLY2:
		printf("MSG_RANGING_REPLY2");
		return;

	case P2PRanging::MSG_RANGING_REPORT:
		printf("MSG_RANGING_REPORT");
		return;

	case P2PRanging::MSG_RANGING_FAILED:
		printf("MSG_RANGING_FAILED");
		return;


	}

}

void P2PRanging::runLoop()
{

	if (_haveUnhandledSentMsg)
	{
		perf_begin(pc_time_loopTxSec);
		_haveUnhandledSentMsg = false;
		switch (_lastTxMsg )//message ID
		{
		case MSG_RANGING_INIT:
			DW1000.getTransmitTimestamp(_timeRangingInitSent);
            noteActivity();
            perf_begin(pc_time_SInit_RReply1);
            perf_end(pc_time_RRange_SInit);
            perf_end(pc_time_LoopRequester);
            perf_begin(pc_time_LoopRequester);
			break;

        case MSG_RANGING_REPLY1:
			DW1000.getTransmitTimestamp(_timeRangingReply1Sent);
            noteActivity();
            perf_end(pc_time_RInit_SReply1);
            perf_begin(pc_time_SReply1_RReply2);
			break;

		case MSG_RANGING_REPLY2:
            DW1000.getTransmitTimestamp(_timeRangingReply2Sent);
            noteActivity();
            perf_end(pc_time_RReply1_SReply2);
            perf_begin(pc_time_SReply2_RRange);
            break;

		case MSG_RANGING_REPORT:
            perf_end(pc_time_RReply2_SRange);
            perf_begin(pc_time_SRange_RInit);
			resetInactive();
            break;

		case MSG_RANGING_FAILED:
			resetInactive();
            break;

		default:
			//???
			//TODO this shouldn't happen. Should log this somehow.
			break;
		}
        perf_end(pc_time_loopTxSec);
		return;
	}

	if (_haveUnhandledReceivedMsg)
	{
		perf_begin(pc_time_loopRxSec);
		_haveUnhandledReceivedMsg = false;
		// get message and parse
		DW1000.getRxData(_rxData, LEN_DATA);
		P2PRanging::MessageTypes msgId = P2PRanging::MessageTypes(_rxData[MSG_FIELD_TYPE]);
		if (msgId != _expectedMsg && msgId != MSG_RANGING_INIT)
		{
			// unexpected message, start over again
			printf("Rx unexpected message! expected <");
			print_message_str(_expectedMsg);
			printf(">, received <");
			print_message_str(msgId);
			printf(">\n");
			_expectedMsg = MSG_RANGING_REPLY1;
            perf_count(pc_count_unexpectedMsg);
            perf_end(pc_time_loopRxSec);
			return;
		}

		switch(msgId){
		case MSG_RANGING_INIT:
			// on MSG_RANGING_INIT we (re-)start, so no protocol failure
			_protocolFailed = false;
			DW1000.getReceiveTimestamp(_timeRangingInitReceived);
			_expectedMsg = MSG_RANGING_REPLY2;
			transmitRangingReply1();
			noteActivity();
            perf_begin(pc_time_RInit_SReply1);
            perf_end(pc_time_SRange_RInit);

            perf_end(pc_time_LoopResponder);
            perf_begin(pc_time_LoopResponder);
            break;
		case MSG_RANGING_REPLY1:
			DW1000.getReceiveTimestamp(_timeRangingReply1Received);
			_expectedMsg = MSG_RANGING_REPORT;
			transmitRangingReply2();
			noteActivity();
            perf_end(pc_time_SInit_RReply1);
            perf_begin(pc_time_RReply1_SReply2);
            break;
		case MSG_RANGING_REPLY2:
			DW1000.getReceiveTimestamp(_timeRangingReply2Received);
			_expectedMsg = MSG_RANGING_INIT;
			if (!_protocolFailed)
			{
				_timeRangingInitSent.setTimestamp(&_rxData[MSG_FIELD_DATA_START]);
				_timeRangingReply1Received.setTimestamp(&_rxData[MSG_FIELD_DATA_START] + 5);
				_timeRangingReply2Sent.setTimestamp(&_rxData[MSG_FIELD_DATA_START] + 10);
				// (re-)compute range as two-way ranging is done
				computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
				float distance = _timeComputedRange.getAsMeters();
                uint32_t range_um = uint32_t(distance*1000000 + 0.5f);
				transmitRangeReport(range_um);//_timeComputedRange.getAsMicroSeconds());
				static float avgDistance = 0;
				static int avgDistanceCount = 0;
				avgDistance += distance;
				avgDistanceCount++;
				// update sampling rate (each second)
				_successRangingCount++;
				int32_t curTime_ms = DW1000Device::getTimeMillis();
				if (curTime_ms - _rangingCountPeriod > 1000)
				{
					_samplingRate = (1000.0f * _successRangingCount)
							/ (curTime_ms - _rangingCountPeriod);
					_rangingCountPeriod = curTime_ms;
					_successRangingCount = 0;
					printf("Range: %dmm\n", int(1000.f * distance + 0.5f));
					printf("Avg. range: %dmm, over %d\n",
							int(1000.f * avgDistance + 0.5f) / avgDistanceCount,
							avgDistanceCount);
					printf("\t RX power: %f dDm\n",
							double(DW1000.getReceivePower()));
					printf("\t Sampling: %d.%03d Hz\n",
							int(0.5f + _samplingRate),
							int(0.5f + 1000 * _samplingRate) % 1000);
					avgDistance = 0;
					avgDistanceCount = 0;
				}
			}
			else
			{
				transmitRangeFailed();
			}

			noteActivity();
            perf_begin(pc_time_RReply2_SRange);
            perf_end(pc_time_SReply1_RReply2);
            break;
		case MSG_RANGING_REPORT:
            perf_count(pc_count_RangingSucceeded);
			_expectedMsg = MSG_RANGING_INIT;
			uint32_t curRange_um;
			memcpy(&curRange_um, &_rxData[MSG_FIELD_DATA_START], 4);
			printf("Received MSG_RANGING_REPORT, dist = %dmm\n", int(curRange_um/1000));
//            startNewRanging = true;
			transmitRangingInit();
			noteActivity();
            perf_end(pc_time_SReply2_RRange);
            perf_begin(pc_time_RRange_SInit);
            break;
		case MSG_RANGING_FAILED:
            perf_count(pc_count_RangingFailed);
			printf("Ranging failed!\n");
			_expectedMsg = MSG_RANGING_INIT;
			transmitRangingInit();
			noteActivity();
			break;

		}

        perf_end(pc_time_loopRxSec);
		return;
	}

	if (DW1000Device::getTimeMillis() - _lastActivityTime_ms > _resetPeriod_ms)
	{
        perf_count(pc_timeout);
		resetInactive();
		if (_autoTxRangingInit)
		{
			transmitRangingInit(); //mwm todo: We should have a nicer way of deciding to initiate a conversation...
		}
		return;
	}
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

void P2PRanging::computeRangeAsymmetric()
{
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (_timeRangingReply1Received - _timeRangingInitSent).wrap();
	DW1000Time reply1 = (_timeRangingReply1Sent - _timeRangingInitReceived).wrap();
	DW1000Time round2 = (_timeRangingReply2Received - _timeRangingReply1Sent).wrap();
	DW1000Time reply2 = (_timeRangingReply2Sent - _timeRangingReply1Received).wrap();
	DW1000Time tof = (round1 * round2 - reply1 * reply2)
			/ (round1 + round2 + reply1 + reply2);
	// set tof timestamp
	_timeComputedRange.setTimestamp(tof);
}

void P2PRanging::computeRangeSymmetric()
{
	// symmetric two-way ranging (less computation intense, more error prone on clock drift)
	DW1000Time tof = ((_timeRangingReply1Received - _timeRangingInitSent)
			- (_timeRangingReply1Sent - _timeRangingInitReceived)
			+ (_timeRangingReply2Received - _timeRangingReply1Sent)
			- (_timeRangingReply2Sent - _timeRangingReply1Received)) * 0.25f;
	// set tof timestamp
	_timeComputedRange.setTimestamp(tof);
}

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */
