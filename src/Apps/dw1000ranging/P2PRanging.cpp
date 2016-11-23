#include "P2PRanging.h"
#include <systemlib/perf_counter.h>

#include "DW1000.h"

using namespace DW1000NS;

//definitions of static members:
volatile P2PRanging::MessageTypes P2PRanging::_expectedMsg;
volatile P2PRanging::MessageTypes P2PRanging::_lastTxMsg;
volatile P2PRanging::MessageTypes P2PRanging::_lastRxMsg;

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
uint16_t P2PRanging::_successRangingCount;
uint32_t P2PRanging::_rangingCountPeriod;
float P2PRanging::_samplingRate;
uint8_t P2PRanging::_myId;
uint8_t P2PRanging::_commPartnerId;
uint8_t P2PRanging::_rangingTargetId;
uint16_t P2PRanging::_networkId;
unsigned P2PRanging::_numRangingsInitiationsSent;
unsigned P2PRanging::_numRangingsInitiationsReceived;
unsigned P2PRanging::_numRangingsCompletedSent;
unsigned P2PRanging::_numRangingsCompletedReceived;
unsigned P2PRanging::_numTimeouts;
unsigned P2PRanging::_numResets;
unsigned P2PRanging::_numMsgReceived;
unsigned P2PRanging::_numMsgSent;
unsigned P2PRanging::_numMsgSentInit;
unsigned volatile P2PRanging::_numIRQReceived;
unsigned volatile P2PRanging::_numIRQSent;

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

	// ranging counter (per second)
	_successRangingCount = 0;
	_rangingCountPeriod = 0;
	_samplingRate = 0;
	_lastActivityTime_ms = 0;

    _rangingTargetId = 0;

    _numRangingsInitiationsSent = 0;
    _numRangingsInitiationsReceived = 0;
    _numRangingsCompletedSent = 0;
    _numRangingsCompletedReceived = 0;
    _numTimeouts = 0;
    _numResets = 0;
    _numMsgReceived = 0;
    _numMsgSent = 0;
    _numMsgSentInit = 0;

    _numIRQReceived = 0;
    _numIRQSent = 0;
}

int P2PRanging::Initialize(uint8_t deviceId, uint16_t networkId)
{
	_myId = deviceId;
    _networkId = networkId;
	// initialize the driver
	DW1000.begin();
	if (DW1000.configure())
	{
		return -1;
	}
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(uint16_t(_myId));
	DW1000.setNetworkId(_networkId);
	DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY); //mwm TODO why do other modes fail???
	DW1000.commitConfiguration();

	DW1000.enableAllLeds();

	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(P2PRanging::handleSent);
	DW1000.attachReceivedHandler(P2PRanging::handleReceived);
//	DW1000.attachErrorHandler(P2PRanging::handleError);
//	DW1000.attachReceiveFailedHandler(P2PRanging::handleReceiveFailed);
//	DW1000.attachReceiveTimeoutHandler(P2PRanging::handleReceiveTimeout);
//	DW1000.attachReceiveTimestampAvailableHandler(P2PRanging::handleReceiveTimestampAvailable);

	DW1000.interruptOnReceived(true);
//	DW1000.interruptOnReceiveFailed(true);
//	DW1000.interruptOnReceiveTimeout(true);
	DW1000.interruptOnSent(true);

	//try:
    DW1000.interruptOnReceiveFailed(false);
    DW1000.interruptOnReceiveTimestampAvailable(false);
    DW1000.interruptOnAutomaticAcknowledgeTrigger(false);
    DW1000.setReceiverAutoReenable(true);

	DW1000.writeSystemEventMaskRegister();
	resetInactive();

	return 0;
}

void P2PRanging::resetInactive()
{
    _numResets++;
    DW1000.idle();

    //TODO: unclear why we need the below, but this helps with the resets...
    DW1000.clearReceiveStatus();
    DW1000.clearReceiveTimestampAvailableStatus();
    DW1000.clearTransmitStatus();
	DW1000.writeSystemEventMaskRegister();

	_expectedMsg = MSG_RANGING_INIT; //listen for this, if we don't know what else is going on.
	noteActivity();
	rangingReceiver();
}

void P2PRanging::clearSysStatus()
{
    DW1000.printStatus();

    DW1000.clearReceiveStatus();
    DW1000.clearReceiveTimestampAvailableStatus();
    DW1000.clearTransmitStatus();
	resetInactive();
    DW1000.printStatus();
}

void P2PRanging::handleSent()
{
	// status change on sent success
	_haveUnhandledSentMsg = true;
	perf_count(pc_TxCount);
	_numIRQSent++;
}

void P2PRanging::handleReceived()
{
	// status change on received success
	_haveUnhandledReceivedMsg = true;
	perf_count(pc_RxCount);
	_numIRQReceived++;
}

void P2PRanging::handleError()
{
	//TODO
}

void P2PRanging::handleReceiveFailed()
{
	//TODO
}

void P2PRanging::handleReceiveTimeout()
{
	//TODO
}

void P2PRanging::handleReceiveTimestampAvailable()
{
	//TODO
}

void P2PRanging::transmitRangingInit()
{
    _commPartnerId = _rangingTargetId;
    _rangingTargetId = 0;

	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_INIT;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_SENDER_ID] = _myId;
	_txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_numMsgSentInit++;
	_expectedMsg = MSG_RANGING_REPLY1;
}

void P2PRanging::transmitRangingReply1()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_REPLY1;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_SENDER_ID] = _myId;
	_txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(DELAY_TIME_US, DW1000Time::MICROSECONDS);
	DW1000.setDelay(deltaTime);
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_numMsgSentInit++;
	_expectedMsg = MSG_RANGING_REPLY2;
}

void P2PRanging::transmitRangingReply2()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_REPLY2;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_SENDER_ID] = _myId;
	_txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
	// delay sending the message and remember expected future sent timestamp
	DW1000Time deltaTime = DW1000Time(DELAY_TIME_US, DW1000Time::MICROSECONDS);
	_timeRangingReply2Sent = DW1000.setDelay(deltaTime);
	_timeRangingInitSent.getTimestamp(&_txData[MSG_FIELD_DATA_START]);
	_timeRangingReply1Received.getTimestamp(&_txData[MSG_FIELD_DATA_START] + 5);
	_timeRangingReply2Sent.getTimestamp(&_txData[MSG_FIELD_DATA_START] + 10);
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_numMsgSentInit++;
	_expectedMsg = MSG_RANGING_REPORT;
}

void P2PRanging::transmitRangeReport(const uint32_t curRange_um)
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_REPORT ;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_SENDER_ID] = _myId;
	_txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
	// write final ranging result
    memcpy(&_txData[MSG_FIELD_DATA_START], &curRange_um, 4);
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_numMsgSentInit++;
	_expectedMsg = MSG_RANGING_INIT;
    //NOTE: cannot reset here, would cancel transmission resetInactive();
}

void P2PRanging::transmitRangeFailed()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_lastTxMsg = MSG_RANGING_FAILED;
	_txData[MSG_FIELD_TYPE] = _lastTxMsg;
	_txData[MSG_FIELD_SENDER_ID] = _myId;
	_txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
	DW1000.setTxData(_txData, LEN_DATA);
	DW1000.startTransmit();
	_numMsgSentInit++;
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

void P2PRanging::printMessageStr(P2PRanging::MessageTypes msg){
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

	default:
		printf("UNKNOWN CODE = <%d>", int(msg));
	}
}

void P2PRanging::runLoop()
{

	if (_haveUnhandledSentMsg)
	{
        _numMsgSent++;
		perf_begin(pc_time_loopTxSec);
		_haveUnhandledSentMsg = false;
		switch (_lastTxMsg )//message ID
		{
		case MSG_RANGING_INIT:
			DW1000.getTransmitTimestamp(_timeRangingInitSent);
            _numRangingsInitiationsSent++;
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
			_numRangingsCompletedSent++;
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
        _numMsgReceived++;
		perf_begin(pc_time_loopRxSec);
		_haveUnhandledReceivedMsg = false;
		// get message and parse
		DW1000.getRxData(_rxData, LEN_DATA);
		P2PRanging::MessageTypes msgId = P2PRanging::MessageTypes(_rxData[MSG_FIELD_TYPE]);

		uint8_t senderId = _rxData[MSG_FIELD_SENDER_ID];
		uint8_t targetId = _rxData[MSG_FIELD_TARGET_ID];

        _lastRxMsg = msgId;
		if(targetId != _myId){
			//not for us.
			//TODO: We should record this message anyway
			return;
		}

		if (msgId != _expectedMsg && msgId != MSG_RANGING_INIT)
		{
			// unexpected message, start over again
//			printf("Rx unexpected message! expected <");
//			printMessageStr(_expectedMsg);
//			printf(">, received <");
//			printMessageStr(msgId);
//			printf(">\n");
			_expectedMsg = MSG_RANGING_INIT;
            perf_count(pc_count_unexpectedMsg);
            perf_end(pc_time_loopRxSec);
			return;
		}

		switch(msgId){
		case MSG_RANGING_INIT:
			// on MSG_RANGING_INIT we (re-)start, so no protocol failure
			_commPartnerId = senderId;
			_protocolFailed = false;
			DW1000.getReceiveTimestamp(_timeRangingInitReceived);
			_expectedMsg = MSG_RANGING_REPLY2;
			transmitRangingReply1();
			noteActivity();
            perf_begin(pc_time_RInit_SReply1);
            perf_end(pc_time_SRange_RInit);

            perf_end(pc_time_LoopResponder);
            perf_begin(pc_time_LoopResponder);
            _numRangingsInitiationsReceived++;
            break;
		case MSG_RANGING_REPLY1:
			if(_commPartnerId != senderId){
				//Something's weird, this is not from the person we were talking to.
                _protocolFailed = true;
                break;
			}
			DW1000.getReceiveTimestamp(_timeRangingReply1Received);
			_expectedMsg = MSG_RANGING_REPORT;
			transmitRangingReply2();
			noteActivity();
            perf_end(pc_time_SInit_RReply1);
            perf_begin(pc_time_RReply1_SReply2);
            break;
		case MSG_RANGING_REPLY2:
			if(_commPartnerId != senderId){
				//Something's weird, this is not from the person we were talking to.
                _protocolFailed = true;
                break;
			}
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
				int32_t curTime_ms = DW1000Class::getCPUTimeMillis();
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
			if(_commPartnerId != senderId){
				//Something's weird, this is not from the person we were talking to.
                _protocolFailed = true;
                break;
			}
            perf_count(pc_count_RangingSucceeded);
			_expectedMsg = MSG_RANGING_INIT;
			uint32_t curRange_um;
			memcpy(&curRange_um, &_rxData[MSG_FIELD_DATA_START], 4);
//			printf("Received MSG_RANGING_REPORT, dist = %dmm\n", int(curRange_um/1000));
			printf("%d,", int(curRange_um/1000));
            if(_rangingTargetId){
                transmitRangingInit();
            }
			noteActivity();
            perf_end(pc_time_SReply2_RRange);
            perf_begin(pc_time_RRange_SInit);
            _numRangingsCompletedReceived++;
            break;
		case MSG_RANGING_FAILED:
            perf_count(pc_count_RangingFailed);
			printf("Ranging failed!\n");
			_expectedMsg = MSG_RANGING_INIT;
            if(_rangingTargetId){
                transmitRangingInit();
            }
			noteActivity();
			break;

		}

        perf_end(pc_time_loopRxSec);
		return;
	}

	if (DW1000Class::getCPUTimeMillis() - _lastActivityTime_ms > RESET_PERIOD_MS)
	{
        perf_count(pc_timeout);
		resetInactive();
        _numTimeouts++;
		if(_rangingTargetId){
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

void P2PRanging::printStatus(){
	printf("\n== P2PRanging status: ==\n");
    printf("\tMy ID = %d, comm partner = %d, ranging target = %d\n", int(_myId), int(_commPartnerId), int(_rangingTargetId));
    printf("\tLast Tx message type = ");
    printMessageStr(_lastTxMsg);
    printf("\n\tLast Rx message type = ");
    printMessageStr(_lastRxMsg);
    printf("\n\t expected next message = ");
    printMessageStr(_expectedMsg);
    printf("\n");
    printf("\tNum ranging initations:  sent = %d, received = %d\n", int(_numRangingsInitiationsSent), int(_numRangingsInitiationsReceived));
    printf("\tNum ranging completions: sent = %d, received = %d\n", int(_numRangingsCompletedSent), int(_numRangingsCompletedReceived));
    printf("\tNum resets = %d, timeouts = %d\n", int(_numTimeouts), int(_numTimeouts));
    printf("\tTotal num msg rx = %d, tx init = %d, tx complete = %d\n", int(_numMsgReceived), int(_numMsgSentInit), int(_numMsgSent));
    printf("\tTotal num IRQ rx = %d, tx = %d\n", int(_numIRQReceived), int(_numIRQSent));
    printf("----DW1000 driver status----\n");
    DW1000.printStatus();
    printf("\n\n");
}
