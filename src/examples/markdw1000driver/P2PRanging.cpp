#include "P2PRanging.h"

#include "DW1000.cpp"

//definitions of static members:
volatile P2PRanging::Status P2PRanging::_status;
volatile P2PRanging::MessageTypes P2PRanging::_expectedMsgId;
volatile bool P2PRanging::_sentAck;
volatile bool P2PRanging::_receivedAck;
bool P2PRanging::_protocolFailed;
DW1000Time P2PRanging::_timePollSent;
DW1000Time P2PRanging::_timePollReceived;
DW1000Time P2PRanging::_timePollAckSent;
DW1000Time P2PRanging::_timePollAckReceived;
DW1000Time P2PRanging::_timeRangeSent;
DW1000Time P2PRanging::_timeRangeReceived;
DW1000Time P2PRanging::_timeComputedRange;
uint8_t P2PRanging::_data[P2PRanging::LEN_DATA];
uint32_t P2PRanging::_lastActivityTime_ms;
uint32_t P2PRanging::_resetPeriod_ms;
uint16_t P2PRanging::_replyDelayTime_us;
uint16_t P2PRanging::_successRangingCount;
uint32_t P2PRanging::_rangingCountPeriod;
float P2PRanging::_samplingRate;
bool P2PRanging::_autoTxPoll;

P2PRanging::P2PRanging()
{
	// message sent/received state
	_sentAck = false;
	_receivedAck = false;
	_protocolFailed = false;

	_status = LISTENING;

	_resetPeriod_ms = DEFAULT_RESET_PERIOD_MS;
	// reply times (same on both sides for symm. ranging)
	_replyDelayTime_us = DEFAULT_DELAY_TIME_US;
	// ranging counter (per second)
	_successRangingCount = 0;
	_rangingCountPeriod = 0;
	_samplingRate = 0;
	_lastActivityTime_ms = 0;

	_autoTxPoll = false;
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
	DW1000.setDeviceAddress(2);
	DW1000.setNetworkId(10);
	DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY); //somewhat arbitrary mode
	DW1000.commitConfiguration();

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
	_expectedMsgId = POLL; //listen for this, if we don't know what else is going on.
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
	_receivedAck = true;
}

void P2PRanging::transmitPoll()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = POLL;
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsgId = POLL_ACK;
}

void P2PRanging::transmitPollAck()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = POLL_ACK;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTime_us,
			DW1000Time::MICROSECONDS);
	DW1000.setDelay(deltaTime);
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsgId = RANGE;
}

void P2PRanging::transmitRange()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = RANGE;
	// delay sending the message and remember expected future sent timestamp
	DW1000Time deltaTime = DW1000Time(_replyDelayTime_us,
			DW1000Time::MICROSECONDS);
	_timeRangeSent = DW1000.setDelay(deltaTime);
	_timePollSent.getTimestamp(_data + 1);
	_timePollAckReceived.getTimestamp(_data + 6);
	_timeRangeSent.getTimestamp(_data + 11);
	DW1000.setData(_data, LEN_DATA);
	DW1000.startTransmit();
	_expectedMsgId = RANGE_REPORT;
}

void P2PRanging::transmitRangeReport(const float curRange)
{
	DW1000.newTransmit();
	DW1000.setDefaults();
	_data[0] = RANGE_REPORT;
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
	_data[0] = RANGE_FAILED;
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
	if (!_sentAck && !_receivedAck)
	{
		// check if inactive
		if (DW1000Device::getTimeMillis() - _lastActivityTime_ms
				> _resetPeriod_ms)
		{
			resetInactive();
			if (_autoTxPoll)
			{
				transmitPoll(); //mwm todo: We should have a nicer way of deciding to initiate a conversation...
			}
		}
		return;
	}
	// continue on any success confirmation
	if (_sentAck)
	{
		_sentAck = false;
		uint8_t msgId = _data[0];
		if (msgId == POLL)
		{
			DW1000.getTransmitTimestamp(_timePollSent);
			noteActivity();
		}
		else if (msgId == POLL_ACK)
		{
			DW1000.getTransmitTimestamp(_timePollAckSent);
			noteActivity();
		}
		else if (msgId == RANGE)
		{
			DW1000.getTransmitTimestamp(_timeRangeSent);
			noteActivity();
		}
		//else?
	}

	if (_receivedAck)
	{
		_receivedAck = false;
		// get message and parse
		DW1000.getData(_data, LEN_DATA);
		uint8_t msgId = _data[0];
		if (msgId != _expectedMsgId)
		{
			// unexpected message, start over again
			_expectedMsgId = POLL_ACK;
			if (_autoTxPoll)
			{
				transmitPoll();
			}
			return;
		}

		if (msgId == POLL)
		{
			// on POLL we (re-)start, so no protocol failure
			_protocolFailed = false;
			DW1000.getReceiveTimestamp(_timePollReceived);
			_expectedMsgId = RANGE;
			transmitPollAck();
			noteActivity();
		}
		else if (msgId == POLL_ACK)
		{
			DW1000.getReceiveTimestamp(_timePollAckReceived);
			_expectedMsgId = RANGE_REPORT;
			transmitRange();
			noteActivity();
		}
		else if (msgId == RANGE_REPORT)
		{
			_expectedMsgId = POLL_ACK;
			float curRange;
			memcpy(&curRange, _data + 1, 4);
			transmitPoll();
			noteActivity();
		}
		else if (msgId == RANGE)
		{
			DW1000.getReceiveTimestamp(_timeRangeReceived);
			_expectedMsgId = POLL;
			if (!_protocolFailed)
			{
				_timePollSent.setTimestamp(_data + 1);
				_timePollAckReceived.setTimestamp(_data + 6);
				_timeRangeSent.setTimestamp(_data + 11);
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
		else if (msgId == RANGE_FAILED)
		{
			_expectedMsgId = POLL_ACK;
			transmitPoll();
			noteActivity();
		}
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
	DW1000Time round1 = (_timePollAckReceived - _timePollSent).wrap();
	DW1000Time reply1 = (_timePollAckSent - _timePollReceived).wrap();
	DW1000Time round2 = (_timeRangeReceived - _timePollAckSent).wrap();
	DW1000Time reply2 = (_timeRangeSent - _timePollAckReceived).wrap();
	DW1000Time tof = (round1 * round2 - reply1 * reply2)
			/ (round1 + round2 + reply1 + reply2);
	// set tof timestamp
	_timeComputedRange.setTimestamp(tof);
}

void P2PRanging::computeRangeSymmetric()
{
	// symmetric two-way ranging (less computation intense, more error prone on clock drift)
	DW1000Time tof = ((_timePollAckReceived - _timePollSent)
			- (_timePollAckSent - _timePollReceived)
			+ (_timeRangeReceived - _timePollAckSent)
			- (_timeRangeSent - _timePollAckReceived)) * 0.25f;
	// set tof timestamp
	_timeComputedRange.setTimestamp(tof);
}

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */
