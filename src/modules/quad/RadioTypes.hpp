#pragma once

//Definitions for use with the radio

namespace RadioTypes {

struct RadioMessageDecoded {
 public:
  enum {
    IDX_FLAGS = 0,
    IDX_BUTTONS = IDX_FLAGS + 1,
    IDX_FLOATS = IDX_BUTTONS + 1,
    RADIO_FLOAT_ENCODED_SIZE = 2,  //number of bytes we use to encode a float
    RADIO_FLOAT_ENCODED_MAX = 1 << (RADIO_FLOAT_ENCODED_SIZE * 8),
    RADIO_FLOAT_ENCODED_HALF = RADIO_FLOAT_ENCODED_MAX / 2,
    NUM_RADIO_FLOAT_FIELDS = 4,
    RAW_PACKET_SIZE = IDX_FLOATS
        + RADIO_FLOAT_ENCODED_SIZE * NUM_RADIO_FLOAT_FIELDS,
  };

  enum {
    MAX_VAL_DEFAULT = 1,
  };

  struct RawMessage {
    uint8_t raw[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  };

  static void encodeToRadioByte(float const valIn, float const limit,
                                unsigned const indx,
                                uint8_t bytes[RAW_PACKET_SIZE]) {
    int out;
    if ((valIn > -limit) && (valIn < limit)) {
      //in acceptable range
      out = int(valIn * RADIO_FLOAT_ENCODED_HALF / limit + 0.5f)
          + RADIO_FLOAT_ENCODED_HALF;
    } else if (valIn > -limit) {
      //max value:
      out = RADIO_FLOAT_ENCODED_MAX - 1;
    } else if (valIn < limit) {
      //min value:
      out = 0;
    } else {
      //NAN:
      out = 0;
    }

    //This is probably super non-portable, and a total hack...
    for (int i = 0; i < RADIO_FLOAT_ENCODED_SIZE; i++) {
      if (indx + i >= RAW_PACKET_SIZE) {
        //make sure we only write in allowed memory
        break;
      }
      bytes[indx + i] = (out >> ((RADIO_FLOAT_ENCODED_SIZE - i - 1) * 8)) % 256;  //the last 8 bits.
    }
    return;
  }

  static float decodeFromRadioBytes(uint8_t const bytesIn[RAW_PACKET_SIZE],
                                    unsigned const indx, float const limit) {
    int out = 0;
    //This is probably super non-portable, and a total hack...
    for (int i = 0; i < RADIO_FLOAT_ENCODED_SIZE; i++) {
      if (indx + i >= RAW_PACKET_SIZE) {
        //make sure we only write in allowed memory
        break;
      }
      out += bytesIn[indx + i] << ((RADIO_FLOAT_ENCODED_SIZE - 1 - i) * 8);
    }
    return limit * (out - RADIO_FLOAT_ENCODED_HALF)
        / float(RADIO_FLOAT_ENCODED_HALF);
  }

  RadioMessageDecoded() {
    flags = 0;
  }

  static inline void CreateCommand(const uint8_t flags,
                                   double const axisValues[4],
                                   bool const buttonRedIn,
                                   bool const buttonYellowIn,
                                   bool const buttonGreenIn,
                                   bool const buttonBlueIn,
                                   bool const buttonStartIn,
                                   bool const buttonSelectIn,
                                   uint8_t rawOut[RAW_PACKET_SIZE]) {
    rawOut[IDX_FLAGS] = flags;
    uint8_t buttons = 0;
    if (buttonRedIn) {
      buttons |= 0x01;
    }
    if (buttonYellowIn) {
      buttons |= 0x02;
    }
    if (buttonGreenIn) {
      buttons |= 0x04;
    }
    if (buttonBlueIn) {
      buttons |= 0x08;
    }
    if (buttonStartIn) {
      buttons |= 0x10;
    }
    if (buttonSelectIn) {
      buttons |= 0x20;
    }

    printf("\nbuttons=%d\n", int(buttons));
    rawOut[IDX_FLAGS] = flags;
    rawOut[IDX_BUTTONS] = buttons;
    for (int i = 0; i < 4; i++) {
      encodeToRadioByte(axisValues[i], MAX_VAL_DEFAULT,
                        IDX_FLOATS + (i) * RADIO_FLOAT_ENCODED_SIZE, rawOut);
    }
  }

  inline RadioMessageDecoded(uint8_t const raw[RAW_PACKET_SIZE]) {
    flags = raw[IDX_FLAGS];
    buttonRed = raw[IDX_BUTTONS] & 0x01;
    buttonYellow = raw[IDX_BUTTONS] & 0x02;
    buttonGreen = raw[IDX_BUTTONS] & 0x04;
    buttonBlue = raw[IDX_BUTTONS] & 0x08;
    buttonStart = raw[IDX_BUTTONS] & 0x10;
    buttonSelect = raw[IDX_BUTTONS] & 0x20;

    for (int i = 0; i < NUM_RADIO_FLOAT_FIELDS; i++) {
      floats[i] = decodeFromRadioBytes(
          raw, IDX_FLOATS + i * RADIO_FLOAT_ENCODED_SIZE, MAX_VAL_DEFAULT);
    }
  }

  uint8_t flags;
  float floats[NUM_RADIO_FLOAT_FIELDS];

  bool buttonRed, buttonYellow, buttonGreen, buttonBlue, buttonStart,
      buttonSelect;

 private:
};

}

