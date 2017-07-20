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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/clock.h>
#include <px4_config.h>
#include <drivers/device/spi.h>

#include "flowdeck.hpp"

static struct spi_dev_s *spi1;

// Disables pushing the flow measurement in the EKF
#define NCS_PIN GPIO_EXPANSION_FLOWDECK_CS

FlowDeck::FlowDeck() {
  _isInit = false;

  _movement_x = 0;
  _movement_y = 0;
  _loopCount = 0;
}

#define FAST_SPI_FREQ (16000000L)
#define SLOW_SPI_FREQ (2000000L)

bool FlowDeck::Init() {
  if (_isInit) {
    return true;
  }
  // Initialize the VL53 sensor using the zRanger deck driver
  //mwm: TODO!
//  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");
//  zRanger->init(NULL);

  sched_lock();
  spi1 = up_spiinitialize(PX4_SPIDEV_EXPANSION_PORT);
  if (!spi1) {
    printf("[boot] FAILED to initialize SPI port 1\r\n");
    return -1;
  }

//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; //~1.35 MHz

//  spiBegin();
//  spiConfigureSlow();
  spi1->ops->setfrequency(spi1, SLOW_SPI_FREQ);
  spi1->ops->setbits(spi1, 8);
  spi1->ops->setmode(spi1, SPIDEV_MODE0);
  sched_unlock();

//  vTaskDelay(M2T(40));
  usleep(40 * 1000);

  stm32_gpiowrite(GPIO_EXPANSION_FLOWDECK_CS, 1);
  //digitalWrite(NCS_PIN, HIGH);
  usleep(2 * 1000);
//  vTaskDelay(M2T(2));
  stm32_gpiowrite(GPIO_EXPANSION_FLOWDECK_CS, 0);
  //digitalWrite(NCS_PIN, LOW);
  usleep(2 * 1000);
//  vTaskDelay(M2T(2));
  stm32_gpiowrite(GPIO_EXPANSION_FLOWDECK_CS, 1);
  //digitalWrite(NCS_PIN, HIGH);
  usleep(2 * 1000);
//  vTaskDelay(M2T(2));

  uint8_t chipId = registerRead(0);
  uint8_t invChipId = registerRead(0x5f);

  printf("Motion chip is: 0x%x\n", chipId);
  printf("si pihc noitoM: 0x%x\n", invChipId);

  // Power on reset
  registerWrite(0x3a, 0x5a);
  usleep(5 * 1000);
  //vTaskDelay(M2T(5));

  // Reading the motion registers one time
  registerRead(0x02);
  registerRead(0x03);
  registerRead(0x04);
  registerRead(0x05);
  registerRead(0x06);
  usleep(1 * 1000);
  //vTaskDelay(M2T(1));

  InitRegisters();

  _isInit = true;

//  xTaskCreate(pamotionTask, "pamotion", 2 * configMINIMAL_STACK_SIZE, NULL,
//  /*priority*/3,
//              NULL);
  return _isInit;
}

void FlowDeck::registerWrite(uint8_t reg, uint8_t value) {
  // Set MSB to 1 for write
  reg |= 0x80u;

  sched_lock();
  spi1->ops->select(spi1, (spi_dev_e) PX4_SPIDEV_EXPANSION_FLOWDECK_DEVID,
  true);
  spi1->ops->exchange(spi1, &reg, &reg, 1);
  spi1->ops->exchange(spi1, &value, &value, 1);
  spi1->ops->select(spi1, (spi_dev_e) PX4_SPIDEV_EXPANSION_FLOWDECK_DEVID,
  false);
  sched_unlock();

//  digitalWrite(NCS_PIN, LOW);
//
//  sleepus(50);
//
//  spiExchange(1, &reg, &reg);
//  sleepus(50);
//  spiExchange(1, &value, &value);
//
//  sleepus(50);
//
//  digitalWrite(NCS_PIN, HIGH);
//  sleepus(200);
}

uint8_t FlowDeck::registerRead(uint8_t reg) {
  uint8_t data = 0;
  uint8_t dummy = 0;

  // Set MSB to 0 for read
  reg &= ~0x80u;

  sched_lock();
  spi1->ops->select(spi1, (spi_dev_e) PX4_SPIDEV_EXPANSION_FLOWDECK_DEVID,
  true);
  spi1->ops->exchange(spi1, &reg, &reg, 1);
  spi1->ops->exchange(spi1, &dummy, &data, 1);
  spi1->ops->select(spi1, (spi_dev_e) PX4_SPIDEV_EXPANSION_FLOWDECK_DEVID,
  false);
  sched_unlock();

//  digitalWrite(NCS_PIN, LOW);
//
//  sleepus(50);
//
//  spiExchange(1, &reg, &reg);
//  sleepus(500);
//  spiExchange(1, &dummy, &data);
//
//  sleepus(50);
//
//  digitalWrite(NCS_PIN, HIGH);
//  sleepus(200);

  return data;
}

void FlowDeck::readMotion(motionBurst_t * motion) {
  uint8_t address = 0x16;

  sched_lock();
  spi1->ops->select(spi1, (spi_dev_e) PX4_SPIDEV_EXPANSION_FLOWDECK_DEVID,
                    true);
  spi1->ops->exchange(spi1, &address, &address, 1);
  spi1->ops->exchange(spi1, (uint8_t*) motion, (uint8_t*) motion,
                      sizeof(motionBurst_t));
  spi1->ops->select(spi1, (spi_dev_e) PX4_SPIDEV_EXPANSION_FLOWDECK_DEVID,
                    false);
  sched_unlock();

//  digitalWrite(NCS_PIN, LOW);
//  sleepus(50);
//  spiExchange(1, &address, &address);
//  sleepus(50);
//  spiExchange(sizeof(motionBurst_t), (uint8_t*) motion, (uint8_t*) motion);
//  sleepus(50);
//  digitalWrite(NCS_PIN, HIGH);
//  sleepus(50);

  uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
  realShutter |= (motion->shutter & 0x0ff) << 8;
  motion->shutter = realShutter;
}

void FlowDeck::InitRegisters() {
  registerWrite(0x7F, 0x00);
  registerWrite(0x61, 0xAD);
  registerWrite(0x7F, 0x03);
  registerWrite(0x40, 0x00);
  registerWrite(0x7F, 0x05);
  registerWrite(0x41, 0xB3);
  registerWrite(0x43, 0xF1);
  registerWrite(0x45, 0x14);
  registerWrite(0x5B, 0x32);
  registerWrite(0x5F, 0x34);
  registerWrite(0x7B, 0x08);
  registerWrite(0x7F, 0x06);
  registerWrite(0x44, 0x1B);
  registerWrite(0x40, 0xBF);
  registerWrite(0x4E, 0x3F);
  registerWrite(0x7F, 0x08);
  registerWrite(0x65, 0x20);
  registerWrite(0x6A, 0x18);
  registerWrite(0x7F, 0x09);
  registerWrite(0x4F, 0xAF);
  registerWrite(0x5F, 0x40);
  registerWrite(0x48, 0x80);
  registerWrite(0x49, 0x80);
  registerWrite(0x57, 0x77);
  registerWrite(0x60, 0x78);
  registerWrite(0x61, 0x78);
  registerWrite(0x62, 0x08);
  registerWrite(0x63, 0x50);
  registerWrite(0x7F, 0x0A);
  registerWrite(0x45, 0x60);
  registerWrite(0x7F, 0x00);
  registerWrite(0x4D, 0x11);
  registerWrite(0x55, 0x80);
  registerWrite(0x74, 0x1F);
  registerWrite(0x75, 0x1F);
  registerWrite(0x4A, 0x78);
  registerWrite(0x4B, 0x78);
  registerWrite(0x44, 0x08);
  registerWrite(0x45, 0x50);
  registerWrite(0x64, 0xFF);
  registerWrite(0x65, 0x1F);
  registerWrite(0x7F, 0x14);
  registerWrite(0x65, 0x67);
  registerWrite(0x66, 0x08);
  registerWrite(0x63, 0x70);
  registerWrite(0x7F, 0x15);
  registerWrite(0x48, 0x48);
  registerWrite(0x7F, 0x07);
  registerWrite(0x41, 0x0D);
  registerWrite(0x43, 0x14);
  registerWrite(0x4B, 0x0E);
  registerWrite(0x45, 0x0F);
  registerWrite(0x44, 0x42);
  registerWrite(0x4C, 0x80);
  registerWrite(0x7F, 0x10);
  registerWrite(0x5B, 0x02);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x41);
  registerWrite(0x70, 0x00);

//  vTaskDelay(M2T(10));  // delay 10ms
  usleep(10 * 1000);

  registerWrite(0x32, 0x44);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x40);
  registerWrite(0x7F, 0x06);
  registerWrite(0x62, 0xF0);
  registerWrite(0x63, 0x00);
  registerWrite(0x7F, 0x0D);
  registerWrite(0x48, 0xC0);
  registerWrite(0x6F, 0xD5);
  registerWrite(0x7F, 0x00);
  registerWrite(0x5B, 0xA0);
  registerWrite(0x4E, 0xA8);
  registerWrite(0x5A, 0x50);
  registerWrite(0x40, 0x80);
}

void FlowDeck::Loop() {
  _loopCount++;
  readMotion(&_currentMotion);

  // Flip motion information to comply with sensor mounting
  // (might need to be changed if mounted diffrently)
  int16_t accpx = -_currentMotion.deltaY;
  int16_t accpy = -_currentMotion.deltaX;

  _movement_x = (float) accpx;
  _movement_y = (float) accpy;
}

//bool FlowDeck::pamotionTest() {
//  if (!isInit) {
//    DEBUG_PRINT("Error while initializing the motion sensor\n");
//  }
//
//  // Test the VL53 driver
//  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");
//
//  return zRanger->test();
//}

void FlowDeck::PrintStatus() {
  printf(" -- FlowDeck -- \n");
  printf("\t\t Is init = %d\n", int(_isInit));
  printf("\t\t Loop count = %d\n", int(_loopCount));
  printf("\t\tMotion x = %f\n", double(_movement_x));
  printf("\t\tMotion y = %f\n", double(_movement_y));
  printf("\t\t_currentMotion: \n");
  printf("\t\t\tmotion = 0x%x\n", int(_currentMotion.motion));
  printf("\t\t\tobservation = %d\n", int(_currentMotion.observation));
  printf("\t\t\tdeltaX = %d\n", int(_currentMotion.deltaX));
  printf("\t\t\tdeltaY = %d\n", int(_currentMotion.deltaY));
  printf("\t\t\tsqual = %d\n", int(_currentMotion.squal));
  printf("\t\t\trawDataSum = %d\n", int(_currentMotion.rawDataSum));
  printf("\t\t\tmaxRawData = %d\n", int(_currentMotion.maxRawData));
  printf("\t\t\tminRawData = %d\n", int(_currentMotion.minRawData));
  printf("\t\t\tshutter = %d\n", int(_currentMotion.shutter));
}
