/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm3_sbp_test.h"
#include <rtcm3_sbp.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define MAX_FILE_SIZE 26000

void sbp_callback_gps (u8 msg_id, u8 length, u8 *buffer, u16 sender_id)
{
  (void) length;
  (void) buffer;
  (void) sender_id;
  static uint32_t msg_count = 0;
  if(msg_count == 3 || msg_count == 19 || msg_count == 41) {
    assert(msg_id == SBP_MSG_BASE_POS_ECEF);
  } else {
    assert(msg_id == SBP_MSG_OBS);
  }
  msg_count++;

//  if(msg_id == SBP_MSG_OBS) {
//    msg_obs_t* sbp_msg = (msg_obs_t*)buffer;
//    printf("Msg Id %u, %u, %u, %u\n",msg_id,(sbp_msg->header.n_obs >> 4), sbp_msg->header.n_obs & 0x0F ,sbp_msg->header.t.tow);
//  }
}

void sbp_callback_glo_day_rollover (u8 msg_id, u8 length, u8 *buffer, u16 sender_id)
{
  (void) length;
  (void) buffer;
  (void) sender_id;
  static uint32_t msg_count = 0;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t* msg = (msg_obs_t*)buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    assert(num_sbp_msgs > 2 );
  }
  msg_count++;

//  if(msg_id == SBP_MSG_OBS) {
//    msg_obs_t* sbp_msg = (msg_obs_t*)buffer;
//    printf("Msg Id %u, %u, %u, %u\n",msg_id,(sbp_msg->header.n_obs >> 4), sbp_msg->header.n_obs & 0x0F ,sbp_msg->header.t.tow);
//  }
}

void test_RTCM3_decode(void)
{
  // This file is GPS only and tests basic decoding functionality
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_gps);
  gps_time_sec_t current_time;
  current_time.wn = 1945;
  current_time.tow = 277500;
  rtcm2sbp_set_gps_time(&current_time,&state);
  rtcm2sbp_set_leap_second(18,&state);

  FILE* fp = fopen("../../tests/data/RTCM3.bin","rb");

  u8 buffer[MAX_FILE_SIZE];

  if(fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], MAX_FILE_SIZE - buffer_index, &state);
    }
    buffer_index++;
  }

  return;
}

void test_day_rollover(void)
{
  // This test contains GPS and GLO obs where the GPS-GLO time difference rolls over a day boundary
  // This test excercises the day rollover time matching code as well as the
  // GPS GLO decoding functionality and time matching

  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_glo_day_rollover);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time,&state);
  rtcm2sbp_set_leap_second(18,&state);

  FILE* fp = fopen("../../tests/data/glo_day_rollover.rtcm","rb");

  u8 buffer[MAX_FILE_SIZE];

  if(fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], MAX_FILE_SIZE - buffer_index, &state);
    }
    buffer_index++;
  }

  return;
}


int main(void)
{
  test_RTCM3_decode();
  test_day_rollover();
}
