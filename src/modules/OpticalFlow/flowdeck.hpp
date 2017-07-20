/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* flowdeck.c: Flow deck driver */
//#include "deck.h"
//#include "debug.h"
//#include "system.h"
//#include "log.h"
//#include "param.h"
//
//#include "FreeRTOS.h"
//#include "task.h"
//
//#include "sleepus.h"
//
//#include "stabilizer_types.h"
//#include "estimator.h"
//#include "estimator_kalman.h"
//
//#include "arm_math.h"
#pragma once

#include <stdlib.h>

typedef struct motionBurst_s {
  union {
    uint8_t motion;
    struct {
      uint8_t frameFrom0 :1;
      uint8_t runMode :2;
      uint8_t reserved1 :1;
      uint8_t rawFrom0 :1;
      uint8_t reserved2 :2;
      uint8_t motionOccured :1;
    };
  };

  uint8_t observation;
  int16_t deltaX;
  int16_t deltaY;

  uint8_t squal;

  uint8_t rawDataSum;
  uint8_t maxRawData;
  uint8_t minRawData;

  uint16_t shutter;
}__attribute__((packed)) motionBurst_t;

class FlowDeck {
 public:
  FlowDeck();

  bool Init();

  void Loop();

  void PrintStatus();

  void GetFlow(int32_t &flowx, int32_t &flowy) const{
    flowx = _movement_x;
    flowy = _movement_y;
  }

 private:
  void registerWrite(uint8_t reg, uint8_t value);
  uint8_t registerRead(uint8_t reg);
  void readMotion(motionBurst_t * motion);
  void InitRegisters();

  void pamotionInit();

  motionBurst_t _currentMotion;

  bool _isInit;

  float _movement_x, _movement_y;
  unsigned _loopCount;

};
