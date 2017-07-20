/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.h: Time-of-flight distance sensor driver
 */

#pragma once

#include <stdint.h>

#define VL53L0X_DEFAULT_ADDRESS 0b0101001

#define VL53L0X_RA_SYSRANGE_START                              0x00

#define VL53L0X_RA_SYSTEM_THRESH_HIGH                          0x0C
#define VL53L0X_RA_SYSTEM_THRESH_LOW                           0x0E

#define VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG                      0x01
#define VL53L0X_RA_SYSTEM_RANGE_CONFIG                         0x09
#define VL53L0X_RA_SYSTEM_INTERMEASUREMENT_PERIOD              0x04

#define VL53L0X_RA_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A

#define VL53L0X_RA_GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR                      0x0B

#define VL53L0X_RA_RESULT_INTERRUPT_STATUS                     0x13
#define VL53L0X_RA_RESULT_RANGE_STATUS                         0x14

#define VL53L0X_RA_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define VL53L0X_RA_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define VL53L0X_RA_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define VL53L0X_RA_RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define VL53L0X_RA_RESULT_PEAK_SIGNAL_RATE_REF                 0xB6

#define VL53L0X_RA_ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28

#define VL53L0X_RA_I2C_SLAVE_DEVICE_ADDRESS                    0x8A

#define VL53L0X_RA_MSRC_CONFIG_CONTROL                         0x60

#define VL53L0X_RA_PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define VL53L0X_RA_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64

#define VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

#define VL53L0X_RA_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define VL53L0X_RA_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62

#define VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52

#define VL53L0X_RA_SYSTEM_HISTOGRAM_BIN                        0x81
#define VL53L0X_RA_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define VL53L0X_RA_HISTOGRAM_CONFIG_READOUT_CTRL               0x55

#define VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define VL53L0X_RA_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20

#define VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP                  0x46

#define VL53L0X_RA_SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define VL53L0X_RA_IDENTIFICATION_MODEL_ID                     0xC0
#define VL53L0X_RA_IDENTIFICATION_REVISION_ID                  0xC2

#define VL53L0X_IDENTIFICATION_MODEL_ID                        0xEEAA
#define VL53L0X_IDENTIFICATION_REVISION_ID                     0x10

#define VL53L0X_RA_OSC_CALIBRATE_VAL                           0xF8

#define VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5

#define VL53L0X_RA_GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define VL53L0X_RA_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define VL53L0X_RA_DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VL53L0X_RA_POWER_MANAGEMENT_GO1_POWER_FORCE            0x80

#define VL53L0X_RA_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

#define VL53L0X_RA_ALGO_PHASECAL_LIM                           0x30
#define VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT                0x30

