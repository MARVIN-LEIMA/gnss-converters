#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <check.h>

#include "nmea.h"

#include "check_suites.h"

/* random helpers */
#define MEMZERO_WITH_SIZEOF(object) \
  memset(&object, 0, sizeof(object));

/* nmea.h specific test helpers */

#define MAX_NMEA_SENTENCE_LENGTH (200u)

static struct {
  char nmea_output_buffer[MAX_NMEA_SENTENCE_LENGTH];
  const char *end;
  char *current;
} nmea_output_s = {
  .nmea_output_buffer = {0},
  .end = nmea_output_s.nmea_output_buffer + MAX_NMEA_SENTENCE_LENGTH,
  .current = nmea_output_s.nmea_output_buffer
};

static void nmea_output_reset(void)
{
  MEMZERO_WITH_SIZEOF(nmea_output_s.nmea_output_buffer);
  nmea_output_s.current = nmea_output_s.nmea_output_buffer;
}

static void nmea_output_add_char(char c)
{
  if (nmea_output_s.current < nmea_output_s.end - 1)
  {
    *(nmea_output_s.current++) = c;
  }
}

static const char* nmea_output_buffer(void) {
  return nmea_output_s.nmea_output_buffer;
}

static size_t nmea_output_length(void) {
  return nmea_output_s.current - nmea_output_s.nmea_output_buffer;
}

void nmea_output(char *s, size_t size)
{
  if (s == NULL) {
    return;
  }

  for (size_t i = 0; i < size; i++) {
    nmea_output_add_char(s[i]);
  }
}

static void check_nmea_setup(void)
{
  nmea_output_reset();
}

#define CHECK_NMEA_OUTPUT_LENGTH_EQ(length) \
  ck_assert_uint_eq(nmea_output_length(), length);

#define CHECK_NMEA_OUTPUT_STREQ(buffer) \
  ck_assert_str_eq(nmea_output_buffer(), buffer);

START_TEST(test_nmea_output)
{
  CHECK_NMEA_OUTPUT_STREQ("");
  CHECK_NMEA_OUTPUT_LENGTH_EQ(0);

  nmea_output_add_char('a');
  CHECK_NMEA_OUTPUT_STREQ("a");

  nmea_output("bcdefg", 6);
  CHECK_NMEA_OUTPUT_STREQ("abcdefg");

  nmea_output_reset();
  nmea_output("bcdefg", 6);
  CHECK_NMEA_OUTPUT_STREQ("bcdefg");
}
END_TEST

// Run after test_nmea_output to validate macros
START_TEST(test_nmea_output_setup)
{
  CHECK_NMEA_OUTPUT_STREQ("");
  CHECK_NMEA_OUTPUT_LENGTH_EQ(0);
}
END_TEST

static msg_pos_llh_t sbp_pos_llh;
static msg_dops_t sbp_dops;
static msg_gps_time_t sbp_msg_time;
static utc_params_t utc_params;
#if 0
static msg_vel_ned_t sbp_vel_ned;
static double propagation_time;
static u8 sender_id;
static msg_baseline_heading_t sbp_baseline_heading;
static u8 n_meas;
static navigation_measurement_t nav_meas[];
#endif

/* end test helpers */

static void check_nmea_gga_setup(void)
{
  check_nmea_setup();
  MEMZERO_WITH_SIZEOF(sbp_pos_llh);
  //MEMZERO_WITH_SIZEOF(sbp_vel_ned);
  MEMZERO_WITH_SIZEOF(sbp_dops);
  MEMZERO_WITH_SIZEOF(sbp_msg_time);
  MEMZERO_WITH_SIZEOF(utc_params);
}

static void check_nmea_gga_teardown(void)
{
}

#define DUMMY_GPGGA_STRING \
  "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F"
START_TEST(test_nmea_gga)
{
  //double propagation_time = 0;
  //u8 sender_id = 0;
  //utc_tm utc_time = {};

  //nmea_gpgga(&sbp_pos_llh,
  //           &sbp_msg_time,
  //           &utc_time,
  //           &sbp_dops,
  //           propagation_time,
  //           sender_id);

  //CHECK_NMEA_OUTPUT_STREQ(DUMMY_GPGGA_STRING);
}
END_TEST

Suite* nmea_suite(void)
{
  Suite *s = suite_create("NMEA");

  TCase *tc_fixture = tcase_create("Fixture");
  tcase_add_checked_fixture(tc_fixture, check_nmea_setup, NULL);
  tcase_add_test(tc_fixture, test_nmea_output);
  tcase_add_test(tc_fixture, test_nmea_output_setup);
  suite_add_tcase(s, tc_fixture);

  TCase *tc_nmea_gga = tcase_create("NMEA GGA");
  tcase_add_checked_fixture(tc_nmea_gga, check_nmea_gga_setup, check_nmea_gga_teardown);
  tcase_add_test(tc_nmea_gga, test_nmea_gga);
  suite_add_tcase(s, tc_nmea_gga);

  return s;
}
