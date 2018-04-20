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

#include <check.h>
#include <stdio.h>

#include  <libswiftnav/constants.h>
#include  <libswiftnav/shm.h>

#include "check_suites.h"

START_TEST(test_shm_gps_decode_shi1)
{
  u32 sf1w3 = 0x3f122c34;
  u8 shi1;

  shm_gps_decode_shi1(sf1w3, &shi1);

  fail_unless(shi1 == 0x2c,
      "shm_gps_decode_shi1() returns 0x%x for 0x%x\n",
      shi1, sf1w3);
}
END_TEST

Suite* shm_suite(void)
{
  Suite *s = suite_create("SHM");

  TCase *tc_core = tcase_create("Core");
  tcase_add_test(tc_core, test_shm_gps_decode_shi1);
  suite_add_tcase(s, tc_core);

  return s;
}
