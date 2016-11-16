#include "P2PRanging.h"

#include "DW1000.h"

using namespace DW1000NS;

//definitions of static members:
volatile P2PRanging::MessageTypes P2PRanging::_expectedMsg;
volatile bool P2PRanging::_sentAck;
volatile bool P2PRanging::_haveUnhandledReceivedMsg;
volatile bool P2PRanging::_protocolFailed;
DW1000Time P2PRanging::_timeRangingInitSent;
DW1000Time P2PRanging::_timeRangingInitReceived;
DW1000Time P2PRanging::_timeRangingReply1Sent;
DW1000Time P2PRanging::_timeRangingReply1Received;
DW1000Time P2PRanging::_timeRangingReply2Sent;
DW1000Time P2PRanging::_timeRangingReply2Received;
DW1000Time P2PRanging::_timeComputedRange;
uint8_t P2PRanging::_data[P2PRanging::LEN_DATA];
volatile uint32_t P2PRanging::_lastActivityTime_ms;
uint32_t P2PRanging::_resetPeriod_ms;
uint16_t P2PRanging::_replyDelayTime_us;
uint16_t P2PRanging::_successRangingCount;
uint32_t P2PRanging::_rangingCountPeriod;
float P2PRanging::_samplingRate;
bool P2PRanging::_autoTxRangingInit;

P2PRanging::P2PRanging()
{
	// message sent/received state
	_sentAck = false;
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

int P2PRanging::Initialize()
{
	// initialize the driver
	DW1000.begin();
	if (DW1000.configure())
		return -1;
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(2);//mwm TODO magic number
	DW1000.setNetworkId(10);//mwm TODO magic number
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
	_sentAck = true;
}

void P2PRanging::handleReceived()
{
	// status change on received success
	_haveUnhandledReceivedMsg = true;
	return;
}

void P2PRanging::transmitRangingInit()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = MSG_RANGING_INIT;
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsg = MSG_RANGING_REPLY1;
}

void P2PRanging::transmitRangingReply1()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = MSG_RANGING_REPLY1;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTime_us,
			DW1000Time::MICROSECONDS);
	DW1000.setDelay(deltaTime);
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsg = MSG_RANGING_REPLY2;
}

void P2PRanging::transmitRange()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = MSG_RANGING_REPLY2;
	// delay sending the message and remember expected future sent timestamp
	DW1000Time deltaTime = DW1000Time(_replyDelayTime_us,
			DW1000Time::MICROSECONDS);
	_timeRangingReply2Sent = DW1000.setDelay(deltaTime);
	_timeRangingInitSent.getTimestamp(_data + 1);
	_timeRangingReply1Received.getTimestamp(_data + 6);
	_timeRangingReply2Sent.getTimestamp(_data + 11);
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsg = MSG_RANGING_REPORT;
}

void P2PRanging::transmitRangeReport(const float curRange)
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = MSG_RANGING_REPORT;
	// write final ranging result
	memcpy(_data + 1, &curRange, 4);
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	resetInactive();
}

void P2PRanging::transmitRangeFailed()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = MSG_RANGING_FAILED;
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	resetInactive();
}

void P2PRanging::rangingReceiver()
{
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}

void P2PRanging::runLoop()
{
	if (_sentAck)
	{
		_sentAck = false;
		uint8_t msgId = _data[0];
		if (msgId == MSG_RANGING_INIT)
		{
			DW1000.getTransmitTimestamp(_timeRangingInitSent);
			noteActivity();
		}
		else if (msgId == MSG_RANGING_REPLY1)
		{
			DW1000.getTransmitTimestamp(_timeRangingReply1Sent);
			noteActivity();
		}
		else if (msgId == MSG_RANGING_REPLY2)
		{
			DW1000.getTransmitTimestamp(_timeRangingReply2Sent);
			noteActivity();
		}
		//else?
		return;
	}

	if (_haveUnhandledReceivedMsg)
	{
		_haveUnhandledReceivedMsg = false;
		// get message and parse
		DW1000.getData(_data, LEN_DATA);
		uint8_t msgId = _data[0];
		if (msgId != _expectedMsg)
		{
			// unexpected message, start over again
			_expectedMsg = MSG_RANGING_REPLY1;
			if (_autoTxRangingInit)
			{
				transmitRangingInit();
			}
			return;
		}

		if (msgId == MSG_RANGING_INIT)
		{
			// on MSG_RANGING_INIT we (re-)start, so no protocol failure
			_protocolFailed = false;
			DW1000.getReceiveTimestamp(_timeRangingInitReceived);
			_expectedMsg = MSG_RANGING_REPLY2;
			transmitRangingReply1();
			noteActivity();
		}
		else if (msgId == MSG_RANGING_REPLY1)
		{
			DW1000.getReceiveTimestamp(_timeRangingReply1Received);
			_expectedMsg = MSG_RANGING_REPORT;
			transmitRange();
			noteActivity();
		}
		else if (msgId == MSG_RANGING_REPORT)
		{
			_expectedMsg = MSG_RANGING_REPLY1;
			float curRange;
			memcpy(&curRange, _data + 1, 4);
			transmitRangingInit();
			noteActivity();
		}
		else if (msgId == MSG_RANGING_REPLY2)
		{
			DW1000.getReceiveTimestamp(_timeRangingReply2Received);
			_expectedMsg = MSG_RANGING_INIT;
			if (!_protocolFailed)
			{
				_timeRangingInitSent.setTimestamp(_data + 1);
				_timeRangingReply1Received.setTimestamp(_data + 6);
				_timeRangingReply2Sent.setTimestamp(_data + 11);
				// (re-)compute range as two-way ranging is done
				computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
				transmitRangeReport(_timeComputedRange.getAsMicroSeconds());
				float distance = _timeComputedRange.getAsMeters();
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
		}
		else if (msgId == MSG_RANGING_FAILED)
		{
			_expectedMsg = MSG_RANGING_REPLY1;
			transmitRangingInit();
			noteActivity();
		}
		return;
	}

	if (DW1000Device::getTimeMillis() - _lastActivityTime_ms > _resetPeriod_ms)
	{
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
