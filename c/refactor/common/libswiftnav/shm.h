/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_SHM_H_
#define LIBSWIFTNAV_SHM_H_

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* GPS Satellite Health Indicators
 * Reference: IS-GPS-200D
 */
typedef struct {
  bool shi1_set; /* SHI1 LNAV SV HEALTH (6 bits, subframe 1, word 3) */
  u8 shi1;

  bool shi4_set; /* SHI4 LNAV alert flag (HOW, bit 18) */
  bool shi4;

  bool shi6_set; /* SHI6 CNAV alert flag (bit 38, each message) */
  bool shi6;
} gps_sat_health_indicators_t;

/* GLO Satellite Health Indicator
 * Reference: GLO ICD 5.1.
 * This is (MSB of B || l).
 */
typedef struct {
  bool shi_set; /* SHI SV HEALTH */
  u8 shi;
} glo_sat_health_indicators_t;

/* BDS Satellite Health Indicator */
typedef struct {
  bool shi_set; /* SHI SV HEALTH */
  u8 shi;
} bds_sat_health_indicators_t;

void shm_gps_decode_shi1(u32 sf1w3, u8* shi1);

bool check_6bit_health_word(const u8 health_bits, const code_t code);

#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */

#endif /* LIBSWIFTNAV_SHM_H_ */
