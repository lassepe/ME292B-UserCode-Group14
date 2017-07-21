/* Telemetry packet header file
 * Author: Eugene Lo
 */
#pragma once

#include <stdlib.h>
#include <string.h>
#include <limits>

namespace TelemetryPacket {
/********************************
 ** GENERAL METHODS/DEFINITIONS **
 ********************************/
enum PacketType {
  PACKET_TYPE_QUAD_TELEMETRY_PT1 = 0,
  PACKET_TYPE_QUAD_TELEMETRY_PT2 = 1,
};

struct data_packet_t {  // Used for sending the packet to the radio
  uint8_t type;
  uint8_t packetNumber;
  uint16_t data[14];
}__attribute__((packed));

struct TelemetryPacket;

float MapToOnesRange(float x, float a, float b);
float MapToAB(float x, float a, float b);
uint16_t EncodeOnesRange(float t);
float DecodeOnesRange(uint16_t t);
void EncodeTelemetryPacket(TelemetryPacket const &src, data_packet_t &out);
void DecodeTelemetryPacket(data_packet_t const &in, TelemetryPacket &out);
void DecodeFloatPacket(data_packet_t const &packetIn, float out[],
                       int const numFloats);

/* Map x from [a,b] to [-1,1] */
float MapToOnesRange(float x, float a, float b) {
  return ((x - a) / (b - a)) * 2 - 1;
}

/* Map x from [-1,1] to [a,b] */
float MapToAB(float x, float a, float b) {
  return ((x + 1) / 2) * (b - a) + a;
}

/* Encode values from range [-1,1] into 2 bytes.
 Input t is mapped to 0 iff t is outside of [-1,1].

 Maximum value of 2 byte uint is 2^16 - 1 = 65535.
 To encode a float from [-1,1], we scale the range up to
 [-2^15-1, 2^15-1] and then offset by +2^15 (since uints are non-negative).
 */
uint16_t EncodeOnesRange(float t) {
  uint16_t out;
  if (t < -1 || t > 1) {
    out = 0;
  } else {
    out = 32768 + 32767 * t;
  }
  return out;
}

/* Decode above encoding */
float DecodeOnesRange(uint16_t t) {
  if (t == 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return (t - 32768) / float(32768);
}

/**********************
 ** TELEMETRY PACKETS **
 **********************/
/* Range limits for packets used in QuadcopterLogic.cpp
 These limits are needed to perform the encoding/decoding from floats to ints
 (see MapToOnesRange, EncodeOnesRange above)
 Limits were determined empirically. */
enum TelemetryRanges {
  TEL_RANGE_GENERIC_MAX = 100,
  TEL_RANGE_GENERIC_MIN = -TEL_RANGE_GENERIC_MAX,

};

struct TelemetryPacket {
  enum {
    NUM_DEBUG_FLOATS = 12,
  };
  // Header Info
  uint8_t type;
  uint8_t packetNumber;  // The ID of the packet, is shared amongst all sub-packets.
  /* seqNum = 0 -> packet includes accel, gyro */
  float accel[3];
  float gyro[3];
  float motorCmds[4];
  float opticalFlowx, opticalFlowy;
  float heightsensor;
  float battVoltage;
  /* seqNum = 1 -> packet includes position, attitude, velocity, panicReason */
  float debugVals[NUM_DEBUG_FLOATS];
  uint8_t debugchar;
};

/* Encode a TelemetryPacket into a data_packet_t */
void EncodeTelemetryPacket(TelemetryPacket const &src, data_packet_t &out) {
  out.type = src.type;
  out.packetNumber = src.packetNumber;
  if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT1) {
    for (int i = 0; i < 3; i++) {
      out.data[i + 0] = EncodeOnesRange(
          MapToOnesRange(src.accel[i], TEL_RANGE_GENERIC_MIN,
                         TEL_RANGE_GENERIC_MAX));
      out.data[i + 3] = EncodeOnesRange(
          MapToOnesRange(src.gyro[i], TEL_RANGE_GENERIC_MIN,
                         TEL_RANGE_GENERIC_MAX));
    }
    for (int i = 0; i < 4; i++) {
      out.data[i + 6] = EncodeOnesRange(
          MapToOnesRange(src.motorCmds[i], TEL_RANGE_GENERIC_MIN,
                         TEL_RANGE_GENERIC_MAX));
    }
    out.data[10] = EncodeOnesRange(
        MapToOnesRange(src.opticalFlowx, TEL_RANGE_GENERIC_MIN,
                       TEL_RANGE_GENERIC_MAX));
    out.data[11] = EncodeOnesRange(
        MapToOnesRange(src.opticalFlowy, TEL_RANGE_GENERIC_MIN,
                       TEL_RANGE_GENERIC_MAX));
    out.data[12] = EncodeOnesRange(
        MapToOnesRange(src.heightsensor, TEL_RANGE_GENERIC_MIN,
                       TEL_RANGE_GENERIC_MAX));

    out.data[13] = EncodeOnesRange(
        MapToOnesRange(src.battVoltage, TEL_RANGE_GENERIC_MIN,
                       TEL_RANGE_GENERIC_MAX));

  } else if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT2) {
    for (int i = 0; i < TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
      out.data[i] = EncodeOnesRange(
          MapToOnesRange(src.debugVals[i], TEL_RANGE_GENERIC_MIN,
                         TEL_RANGE_GENERIC_MAX));
    }

    memcpy(&out.data[12], &src.debugchar, 1);
  }
  return;
}

/* Decode a data_packet_t into a TelemetryPacket */
void DecodeTelemetryPacket(data_packet_t const &in, TelemetryPacket &out) {
  out.type = in.type;
  out.packetNumber = in.packetNumber;
  if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT1) {
    for (int i = 0; i < 3; i++) {
      out.accel[i] = MapToAB(DecodeOnesRange(in.data[i + 0]),
                             TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
      out.gyro[i] = MapToAB(DecodeOnesRange(in.data[i + 3]),
                            TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    }
    for (int i = 0; i < 4; i++) {
      out.motorCmds[i] = MapToAB(DecodeOnesRange(in.data[i + 6]),
                                 TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    }
    out.opticalFlowx = MapToAB(DecodeOnesRange(in.data[10]),
                               TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    out.opticalFlowy = MapToAB(DecodeOnesRange(in.data[11]),
                               TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    out.heightsensor = MapToAB(DecodeOnesRange(in.data[12]),
                               TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    out.battVoltage = MapToAB(DecodeOnesRange(in.data[13]),
                              TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);

  } else if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT2) {
    for (int i = 0; i < TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
      out.debugVals[i] = MapToAB(DecodeOnesRange(in.data[i]),
                                 TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    }
    memcpy(&out.debugchar, &in.data[12], 1);
  }
  return;
}

/******************
 ** FLOAT PACKETS **
 ******************/
/* Reverts uints to floats in range [-1,1] */
void DecodeFloatPacket(data_packet_t const &packetIn, float out[],
                       int const numFloats) {
  for (int i = 0; i < numFloats; i++) {
    if (i >= 14)
      break;
    out[i] = DecodeOnesRange(packetIn.data[i]);
  }
}

}
