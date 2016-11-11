/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000Mac.cpp
 * Arduino global library (header file) working with the DW1000 library
 * for the Decawave DW1000 UWB transceiver IC. This class has the purpose
 * to generate the mac layer
 * 
 * @todo everything, this class is only a prototype
 */

#include "DW1000Mac.h" 
#include "DW1000Ranging.h"

//Constructor and destructor

DW1000Mac::DW1000Mac() {
	_seqNumber = 0;
}


DW1000Mac::~DW1000Mac() {
}

//for poll message we use just 2 bytes address
//total=12 bytes
void DW1000Mac::generateBlinkFrame(uint8_t frame[], uint8_t sourceAddress[], uint8_t sourceShortAddress[]) {
	//Frame Control
	*frame     = FC_1_BLINK;
	//sequence number
	*(frame+1) = _seqNumber;
	//tag 64 bit ID (8 bytes address) -- reverse
	uint8_t sourceAddressReverse[8];
	reverseArray(sourceAddressReverse, sourceAddress, 8);
	memcpy(frame+2, sourceAddressReverse, 8);
	
	//tag 2bytes address:
	uint8_t sourceShortAddressReverse[2];
	reverseArray(sourceShortAddressReverse, sourceShortAddress, 2);
	memcpy(frame+10, sourceShortAddressReverse, 2);
	
	//we increment seqNumber
	incrementSeqNumber();
}

//the short fram usually for Resp, Final, or Report
//2 bytes for Desination Address and 2 bytes for Source Address
//total=9 bytes
void DW1000Mac::generateShortMACFrame(uint8_t frame[], uint8_t sourceShortAddress[], uint8_t destinationShortAddress[]) {
	//Frame controle
	*frame     = FC_1;
	*(frame+1) = FC_2_SHORT;
	//sequence number (11.3) modulo 256
	*(frame+2) = _seqNumber;
	//PAN ID
	*(frame+3) = 0xCA;
	*(frame+4) = 0xDE;
	
	
	//destination address (2 bytes)
	uint8_t destinationShortAddressReverse[2];
	reverseArray(destinationShortAddressReverse, destinationShortAddress, 2);
	memcpy(frame+5, destinationShortAddressReverse, 2);
	
	//source address (2 bytes)
	uint8_t sourceShortAddressReverse[2];
	reverseArray(sourceShortAddressReverse, sourceShortAddress, 2);
	memcpy(frame+7, sourceShortAddressReverse, 2);
	
	
	//we increment seqNumber
	incrementSeqNumber();
}

//the long frame for Ranging init
//8 bytes for Destination Address and 2 bytes for Source Address
//total=15
void DW1000Mac::generateLongMACFrame(uint8_t frame[], uint8_t sourceShortAddress[], uint8_t destinationAddress[]) {
	//Frame controle
	*frame     = FC_1;
	*(frame+1) = FC_2;
	//sequence number
	*(frame+2) = _seqNumber;
	//PAN ID (0xDECA)
	*(frame+3) = 0xCA;
	*(frame+4) = 0xDE;
	
	//destination address (8 bytes) - we need to reverse the uint8_t array
	uint8_t destinationAddressReverse[8];
	reverseArray(destinationAddressReverse, destinationAddress, 8);
	memcpy(frame+5, destinationAddressReverse, 8);
	
	//source address (2 bytes)
	uint8_t sourceShortAddressReverse[2];
	reverseArray(sourceShortAddressReverse, sourceShortAddress, 2);
	memcpy(frame+13, sourceShortAddressReverse, 2);
	
	//we increment seqNumber
	incrementSeqNumber();
}


void DW1000Mac::decodeBlinkFrame(uint8_t frame[], uint8_t address[], uint8_t shortAddress[]) {
	//we save the long address of the sender into the device. -- reverse direction
	uint8_t reverseAddress[8];
	memcpy(reverseAddress, frame+2, 8);
	reverseArray(address, reverseAddress, 8);
	
	uint8_t reverseShortAddress[2];
	memcpy(reverseShortAddress, frame+10, 2);
	reverseArray(shortAddress, reverseShortAddress, 2);
}

void DW1000Mac::decodeShortMACFrame(uint8_t frame[], uint8_t address[]) {
	uint8_t reverseAddress[2];
	memcpy(reverseAddress, frame+7, 2);
	reverseArray(address, reverseAddress, 2);
	//we grab the destination address for the mac frame
	//uint8_t destinationAddress[2];
	//memcpy(destinationAddress, frame+5, 2);
}

void DW1000Mac::decodeLongMACFrame(uint8_t frame[], uint8_t address[]) {
	uint8_t reverseAddress[2];
	memcpy(reverseAddress, frame+13, 2);
	reverseArray(address, reverseAddress, 2);
	//we grab the destination address for the mac frame
	//uint8_t destinationAddress[8];
	//memcpy(destinationAddress, frame+5, 8);
}


void DW1000Mac::incrementSeqNumber() {
	// normally overflow of uint8 automatically resets to 0 if over 255
	// but if-clause seems safer way
	if(_seqNumber == 255)
		_seqNumber = 0;
	else
		_seqNumber++;
}

void DW1000Mac::reverseArray(uint8_t to[], uint8_t from[], int16_t size) {
	for(int16_t i = 0; i < size; i++) {
		*(to+i) = *(from+size-i-1);
	}
}
