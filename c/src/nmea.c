/*
 * Copyright (C) 2013-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/array_tools.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/observation.h>

//#include "board/nap/track_channel.h"
//#include "calc_pvt_me.h"
//#include "io_support.h"
//#include "main.h"
//#include "piksi_systime.h"
//#include "sbp.h"
//#include "sbp_utils.h"
//#include "settings/settings.h"
//#include "starling_threads.h"
//#include "timing/timing.h"
//#include "track/track_sid_db.h"

#include "nmea.h"

#if 0
static u32 gpgga_msg_rate = 1; /* By design GGA should be output at the
                                  solution rate. */
static u32 gpgsv_msg_rate = 10;
static u32 gprmc_msg_rate = 10;
static u32 gpvtg_msg_rate = 1;
static u32 gphdt_msg_rate = 1;
static u32 gpgll_msg_rate = 10;
static u32 gpzda_msg_rate = 10;
static u32 gsa_msg_rate = 10;
#endif

/** \addtogroup io
 * \{ */

/** \defgroup nmea NMEA
 * Send messages in NMEA 2.30 format.
 * \{ */

/* GLO NMEA SV IDs are from 65 to 96 */
#define NMEA_SV_ID_OFFSET_GLO (64)

/* SBAS NMEA SV IDs are from 33 to 54 */
#define NMEA_SV_ID_OFFSET_SBAS (-87)

/* Max SVs reported per GSA message */
#define GSA_MAX_SV 12

/* Number of decimals in NMEA time stamp (valid values 1-4) */
#define NMEA_UTC_S_DECIMALS 2
#define NMEA_UTC_S_FRAC_DIVISOR pow(10, NMEA_UTC_S_DECIMALS)

/* Adequate until end of year 999999.
   Worst case: "%02d%02d%05.2f,%02d,%02d,%lu" */
#define NMEA_TS_MAX_LEN (21 + NMEA_UTC_S_DECIMALS)

/* Accuracy of Course Over Ground */
#define NMEA_COG_DECIMALS 1
#define NMEA_COG_FRAC_DIVISOR pow(10, NMEA_COG_DECIMALS)

/* Based on testing calculated Course Over Ground starts deviating noticeably
 * below this limit. */
#define NMEA_COG_STATIC_LIMIT_MS 0.1f
#define NMEA_COG_STATIC_LIMIT_KNOTS MS2KNOTS(NMEA_COG_STATIC_LIMIT_MS, 0, 0)
#define NMEA_COG_STATIC_LIMIT_KPH MS2KMHR(NMEA_COG_STATIC_LIMIT_MS, 0, 0)

#define NMEA_SUFFIX_LEN                    \
  6 /* How much room to leave for the NMEA \
       checksum, CRLF + null termination,  \
       i.e. "*%02X\r\n\0" */

/** Some helper macros for functions generating NMEA sentences. */

/** NMEA_SENTENCE_START: declare a buffer and set up some pointers
 * max_len = max possible length of the body of the message
 * (not including suffix)
 */
#define NMEA_SENTENCE_START(max_len)            \
  char sentence_buf[max_len + NMEA_SUFFIX_LEN]; \
  char *sentence_bufp = sentence_buf;           \
  char *const sentence_buf_end = sentence_buf + max_len;

/** NMEA_SENTENCE_PRINTF: use like printf, can use multiple times
    within a sentence. */
#define NMEA_SENTENCE_PRINTF(fmt, ...)                                        \
  do {                                                                        \
    sentence_bufp += snprintf(                                                \
        sentence_bufp, sentence_buf_end - sentence_bufp, fmt, ##__VA_ARGS__); \
    if (sentence_bufp >= sentence_buf_end) sentence_bufp = sentence_buf_end;  \
  } while (0)

/** NMEA_SENTENCE_DONE: append checksum and dispatch.
 * \note According to section 5.3.1 of the NMEA 0183 spec, sentences are
 *       terminated with <CR><LF>. The sentence_buf is null_terminated.
 *       The call to nmea_output has been modified to remove the NULL.
 *       This will also affect all registered dispatchers
 */
#define NMEA_SENTENCE_DONE()                                         \
  do {                                                               \
    if (sentence_bufp == sentence_buf_end)                           \
      log_warn("NMEA %.6s cut off", sentence_buf);                   \
    nmea_append_checksum(sentence_buf, sizeof(sentence_buf));        \
    nmea_output(sentence_buf,                                        \
                sentence_bufp - sentence_buf + NMEA_SUFFIX_LEN - 1); \
  } while (0)

/** Output NMEA sentence.

 * \param s The NMEA sentence to output.
 * \param size This is the C-string size, not including the null character
 */
#if 0
static void nmea_output(char *s, size_t size) {
  (void)s;
  (void)size;
  // implement generally
}
#else
void nmea_output(char *s, size_t size);
#endif

bool send_heading = false;
void nmea_setup(void) {
#if 0
  SETTING("nmea", "gpgga_msg_rate", gpgga_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgsv_msg_rate", gpgsv_msg_rate, TYPE_INT);
  SETTING("nmea", "gprmc_msg_rate", gprmc_msg_rate, TYPE_INT);
  SETTING("nmea", "gpvtg_msg_rate", gpvtg_msg_rate, TYPE_INT);
  SETTING("nmea", "gphdt_msg_rate", gphdt_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgll_msg_rate", gpgll_msg_rate, TYPE_INT);
  SETTING("nmea", "gpzda_msg_rate", gpzda_msg_rate, TYPE_INT);
  SETTING("nmea", "gsa_msg_rate", gsa_msg_rate, TYPE_INT);
#endif
}

/** Calculate and append the checksum of an NMEA sentence.
 * Calculates the bitwise XOR of the characters in a string until the end of
 * the string or a `*` is encountered. If the first character is `$` then it
 * is skipped.
 *
 * \param s A null-terminated NMEA sentence, up to and optionally
 * including the '*'
 *
 * \param size Length of the buffer.
 *
 */
static void nmea_append_checksum(char *s, size_t size) {
  u8 sum = 0;
  char *p = s;

  /* '$' header not included in checksum calculation */
  if (*p == '$') {
    p++;
  }

  /* '*'  not included in checksum calculation */
  while (*p != '*' && *p && p + NMEA_SUFFIX_LEN < s + size) {
    sum ^= *p;
    p++;
  }

  sprintf(p, "*%02X\r\n", sum);
}

/** Wrapper function for vsnprintf to accommodate return value handling. Updates
 *  buf_ptr accordingly if there's no encoding error. If buffer is full, sets
 *  buf_ptr to buf_end.
 */
static void vsnprintf_wrap(char **buf_ptr,
                           char *buf_end,
                           const char *format,
                           ...) {
#if 1
  (void)buf_ptr;
  (void)buf_end;
  (void)format;
#else
  va_list args;
  va_start(args, format);

  int res = vsnprintf(*buf_ptr, buf_end - *buf_ptr, format, args);

  va_end(args);

  if (res < 0) {
    log_warn("Error %d in vsnprintf_wrap()", res);
    return;
  }

  *buf_ptr += res;

  if (*buf_ptr >= buf_end) {
    log_warn("Buffer overflow in vsnprintf_wrap()");
    *buf_ptr = buf_end;
  }
#endif
}

/** Generate UTC date time string. Time field is before date field.
 *
 * \param[in] sbp_msg_time Time and date to create the str from.
 * \param[in] time Time field is to be added.
 * \param[in] date Date field is to be added.
 * \param[in] trunc_date Truncate date field. No effect if param date is false.
 * \param[in] t UTC time structure
 * \param[out] utc_str Created date time string.
 * \param[in] size utc_str size.
 *
 */
static void get_utc_time_string(const msg_gps_time_t *sbp_msg_time,
                                bool time,
                                bool date,
                                bool trunc_date,
                                const utc_tm *t,
                                char *utc_str,
                                u8 size) {
  char *buf_end = utc_str + size;

  if ((sbp_msg_time->flags & TIME_SOURCE_MASK) == NO_TIME) {
    if (time) {
      vsnprintf_wrap(&utc_str, buf_end, ","); /* Time (UTC) */
    }

    if (date) {
      vsnprintf_wrap(
          &utc_str, buf_end, trunc_date ? "," : ",,,"); /* Date Stamp */
    }

    return;
  }

  if (time) {
    /* Time (UTC) */
    vsnprintf_wrap(&utc_str,
                   buf_end,
                   "%02u%02u%02u.%0*u,",
                   t->hour,
                   t->minute,
                   t->second_int,
                   NMEA_UTC_S_DECIMALS,
                   (u16)roundf(NMEA_UTC_S_FRAC_DIVISOR * t->second_frac));
  }

  if (date) {
    /* Date Stamp */

    if (trunc_date) {
      vsnprintf_wrap(&utc_str,
                     buf_end,
                     "%02u%02u%02u,",
                     t->month_day,
                     t->month,
                     (u8)(t->year % 100));
    } else {
      vsnprintf_wrap(&utc_str,
                     buf_end,
                     "%02u,%02u,%" PRIu32 ",",
                     t->month_day,
                     t->month,
                     t->year);
    }
  }
}

/* General note: the NMEA functions below mask the time source, position and
   velocity modes to ensure prevention of accidental bugs in the future.

   The SBP specification only specifies the first 3 bits to indicate the mode.
   Thus the RAIM repair flag or other future flags need to be masked out.

   It is needed in this code becasue we pass SBP formatted structures, not the
   raw positioning modes.
*/

/** Assemble a NMEA GPGGA message and send it out NMEA USARTs.
 * NMEA GPGGA message contains Global Positioning System Fix Data.
 *
 * \param sbp_pos_llh       SBP LLH position messages
 * \param sbp_msg_time      SBP GPS Time message
 * \param utc_time          Pointer to UTC time
 * \param sbp_dops          SBP DOP Message for this epoch
 * \param propagation_time  Age of differential corrections [s].
 * \param station_id        Differential reference station ID.
 */
void nmea_gpgga(const msg_pos_llh_t *sbp_pos_llh,
                const msg_gps_time_t *sbp_msg_time,
                const utc_tm *utc_time,
                const msg_dops_t *sbp_dops,
                double propagation_time,
                u8 station_id) {
  /* GGA sentence is formed by splitting latitude and longitude
     into degrees and minutes parts and then printing them separately
     using printf. Before doing the split we want to take care of
     the proper rounding. Doing it after the split would lead to the need
     of handling the case of minutes part overflow separately. Otherwise,
     printf would do the rounding of the minutes part, which could result
     in printing 60 minutes value.
     E.g. doing this way lat = 15.9999999996 would be printed as
     $GPGGA,hhmmss.ss,1600.000000,...
     and NOT
     $GPGGA,hhmmss.ss,1560.000000,...
   */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  u8 fix_type = NMEA_GGA_QI_INVALID;
  if ((sbp_msg_time->flags & POSITION_MODE_MASK) != NO_POSITION) {
    fix_type = get_nmea_quality_indicator(sbp_pos_llh->flags);
  }

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGGA,");

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, false, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  if (fix_type != NMEA_GGA_QI_INVALID) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,",
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,");
  }

  NMEA_SENTENCE_PRINTF("%01d,", fix_type);

  if (fix_type != NMEA_GGA_QI_INVALID) {
    NMEA_SENTENCE_PRINTF("%02d,%.1f,%.2f,M,0.0,M,",
                         sbp_pos_llh->n_sats,
                         round(10 * sbp_dops->hdop * 0.01) / 10,
                         sbp_pos_llh->height);
  } else {
    NMEA_SENTENCE_PRINTF(",,,M,,M,");
  }

  if ((fix_type == NMEA_GGA_QI_DGPS) || (fix_type == NMEA_GGA_QI_FLOAT) ||
      (fix_type == NMEA_GGA_QI_RTK)) {
    NMEA_SENTENCE_PRINTF("%.1f,%04d",
                         propagation_time,
                         station_id & 0x3FF); /* ID range is 0000 to 1023 */
  } else {
    NMEA_SENTENCE_PRINTF(",");
  }

  NMEA_SENTENCE_DONE();
}

int gsa_cmp(const void *a, const void *b) { return (*(u8 *)a - *(u8 *)b); }

static u8 nmea_get_id(const gnss_signal_t sid) {
  u8 id = -1;

  switch (sid_to_constellation(sid)) {
    case CONSTELLATION_GPS:
    case CONSTELLATION_BDS2:
    case CONSTELLATION_GAL:
      id = sid.sat;
      break;
    case CONSTELLATION_GLO:
      id = NMEA_SV_ID_OFFSET_GLO + sid.sat;
      break;
    case CONSTELLATION_SBAS:
      id = NMEA_SV_ID_OFFSET_SBAS + sid.sat;
      break;
    case CONSTELLATION_QZS:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_INVALID:
    default:
      log_error("NMEA: Unsupported constellation");
      break;
  }

  return id;
}

/** Print a NMEA GSA string and send it out NMEA USARTs.
 * NMEA GSA message contains GNSS DOP and Active Satellites.
 *
 * \param prns         Array of PRNs to output.
 * \param num_prns     Number of valid PRNs in array.
 * \param sbp_pos_llh  Pos data pointer.
 * \param sbp_dops     Pointer to SBP MSG DOP struct (PDOP, HDOP, VDOP).
 * \param talker       Talker ID to use.
 */
void nmea_gsa_print(u8 *prns,
                    const u8 num_prns,
                    const msg_pos_llh_t *sbp_pos_llh,
                    const msg_dops_t *sbp_dops,
                    const char *talker) {
  assert(prns);
  assert(sbp_dops);
  assert(sbp_pos_llh);

  bool fix = NO_POSITION != (sbp_pos_llh->flags & POSITION_MODE_MASK);

  /* Our fix is always 3D */
  char fix_mode = fix ? '3' : '1';

  NMEA_SENTENCE_START(120);
  /* Always automatic mode */
  NMEA_SENTENCE_PRINTF("$%sGSA,A,%c,", talker, fix_mode);

  qsort(prns, num_prns, sizeof(u8), gsa_cmp);

  for (u8 i = 0; i < GSA_MAX_SV; i++) {
    if (i < num_prns) {
      NMEA_SENTENCE_PRINTF("%02d,", prns[i]);
    } else {
      NMEA_SENTENCE_PRINTF(",");
    }
  }

  if (fix && (NULL != sbp_dops)) {
    NMEA_SENTENCE_PRINTF("%.1f,%.1f,%.1f",
                         round(sbp_dops->pdop * 0.1) / 10,
                         round(sbp_dops->hdop * 0.1) / 10,
                         round(sbp_dops->vdop * 0.1) / 10);
  } else {
    NMEA_SENTENCE_PRINTF(",,");
  }

  NMEA_SENTENCE_DONE();
}

/** Group measurements by constellation and forward information to GSA
 *  printing function.
 *
 * \note NMEA 0183 - Standard For Interfacing Marine Electronic Devices
 *       versions 2.30, 3.01 and 4.10 state following:
 *       If only GPS, GLONASS, Galileo etc. is used for the reported position
 *       solution the talker ID is GP, GL, etc. and the DOP values pertain to
 *       the individual system. If GPS, GLONASS, Galileo etc. are combined to
 *       obtain the reported position solution multiple GSA sentences are
 *       produced, one with the GPS satellites, another with the GLONASS
 *       satellites and another with Galileo satellites, etc. Each of these GSA
 *       sentences shall have talker ID GN, to indicate that the satellites are
 *       used in a combined solution and each shall have the PDOP, HDOP and VDOP
 *       for the combined satellites used in the position.
 *
 * \param sbp_pos      Pointer to position data
 * \param sbp_dops     Pointer to DOPS data
 * \param n_meas       Number of measurements
 * \param nav_meas     Array of navigation measurements
 */
void nmea_gsa(const msg_pos_llh_t *sbp_pos,
                     const msg_dops_t *sbp_dops,
                     const u8 n_meas,
                     const navigation_measurement_t nav_meas[]) {
  assert(sbp_pos);
  assert(sbp_dops);
  assert(nav_meas);

  u8 prns_gp[GSA_MAX_SV];
  u8 num_prns_gp = 0;
  u8 prns_gl[GSA_MAX_SV];
  u8 num_prns_gl = 0;
  u8 prns_bd[GSA_MAX_SV];
  u8 num_prns_bd = 0;

  /* Assemble list of currently active SVs */
  for (u8 i = 0; i < n_meas; i++) {
    const navigation_measurement_t info = nav_meas[i];
    /* Check following:
     *   - constellation to group by correct talker ID
     *       * GPS and SBAS use GP
     *       * GLO uses GL
     *       * BDS uses BD
     *       * If at least 2 of GP, GL or BD are present, replace them with GN
     *   - maximum group size is GSA_MAX_SV
     *   - if SV is reported already by another signal (eg. GPS L1CA vs L2C)
     */
    if ((IS_GPS(info.sid) || IS_SBAS(info.sid)) && num_prns_gp < GSA_MAX_SV &&
        !is_value_in_array(prns_gp, num_prns_gp, nmea_get_id(info.sid))) {
      prns_gp[num_prns_gp++] = nmea_get_id(info.sid);
      continue;
    }

    if (IS_GLO(info.sid) && num_prns_gl < GSA_MAX_SV &&
        !is_value_in_array(prns_gl, num_prns_gl, nmea_get_id(info.sid))) {
      prns_gl[num_prns_gl++] = nmea_get_id(info.sid);
      continue;
    }

    if (IS_BDS2(info.sid) && num_prns_bd < GSA_MAX_SV &&
        !is_value_in_array(prns_bd, num_prns_bd, nmea_get_id(info.sid))) {
      prns_bd[num_prns_bd++] = nmea_get_id(info.sid);
      continue;
    }
  }

  if (0 == num_prns_gp && 0 == num_prns_gl && 0 == num_prns_bd) {
    /* At bare minimum, print empty GPGSA and be done with it */
    nmea_gsa_print(prns_gp, num_prns_gp, sbp_pos, sbp_dops, "GP");
    return;
  }

  bool use_gn = false;

  u8 constellations = 0;
  constellations += (0 != num_prns_gp) ? 1 : 0;
  constellations += (0 != num_prns_gl) ? 1 : 0;
  constellations += (0 != num_prns_bd) ? 1 : 0;

  if (constellations >= 2) {
    /* At least two constellations detected, use GN talker ID */
    use_gn = true;
  }

  /* Print active GP identified SVs */
  if (0 != num_prns_gp) {
    nmea_gsa_print(
        prns_gp, num_prns_gp, sbp_pos, sbp_dops, use_gn ? "GN" : "GP");
  }

  /* Print active GL identified SVs */
  if (0 != num_prns_gl) {
    nmea_gsa_print(
        prns_gl, num_prns_gl, sbp_pos, sbp_dops, use_gn ? "GN" : "GL");
  }

  /* Print active BD identified SVs */
  if (0 != num_prns_bd) {
    nmea_gsa_print(
        prns_bd, num_prns_bd, sbp_pos, sbp_dops, use_gn ? "GN" : "BD");
  }
}

/** Helper function for nmea_gsv for comparing sids.
 *
 * \param[in] a     ptr to left side sid
 * \param[in] b     ptr to right side sid
 *
 * Return values:
 *   <0 The element pointed to by a goes before the element pointed to by b
 *   0  The element pointed to by a is equivalent to the element pointed to by b
 *   >0 The element pointed to by a goes after the element pointed to by b
 */
int compare_ch_meas(const void *a, const void *b) {
  const channel_measurement_t **ca = (const channel_measurement_t **)a;
  const channel_measurement_t **cb = (const channel_measurement_t **)b;

  return nmea_get_id((*ca)->sid) - nmea_get_id((*cb)->sid);
}

/** Print a NMEA GSV message string and send it out NMEA USARTs.
 * NMEA GSV  message contains GNSS Satellites In View (in this case observed).
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of ch_measurement structs from SVs in track
 * \param[in] talker      indicator which talker ID to use
 */
static void nmea_gsv_print(const u8 n_used,
                           const channel_measurement_t *ch_meas[],
                           const constellation_t talker) {
  char *gnss_s = "";
  if (CONSTELLATION_GPS == talker || CONSTELLATION_SBAS == talker) {
    gnss_s = "GPGSV";
  } else if (CONSTELLATION_GLO == talker) {
    gnss_s = "GLGSV";
  } else if (CONSTELLATION_BDS2 == talker) {
    gnss_s = "BDGSV";
  } else if (CONSTELLATION_GAL == talker) {
    gnss_s = "GAGSV";
  } else {
    log_error("NMEA: Unsupported GNSS type");
    return;
  }

  qsort(ch_meas, n_used, sizeof(channel_measurement_t *), compare_ch_meas);

  u8 n_messages = (n_used + 3) / 4;

  u8 n = 0;

  for (u8 i = 0; i < n_messages; i++) {
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF("$%s,%u,%u,%02u", gnss_s, n_messages, i + 1, n_used);

    for (u8 j = 0; j < 4 && n < n_used; n++) {
#if 0
      s8 ele = track_sid_db_elevation_degrees_get(ch_meas[n]->sid);
      u16 azi = track_sid_db_azimuth_degrees_get(ch_meas[n]->sid);
#endif

      u16 sv_id = nmea_get_id(ch_meas[n]->sid);

      NMEA_SENTENCE_PRINTF(",%02u", sv_id);

#if 0
      if (TRACKING_ELEVATION_UNKNOWN == ele) {
        NMEA_SENTENCE_PRINTF(",");
      } else {
        NMEA_SENTENCE_PRINTF(",%02d", ele);
      }

      if (TRACKING_AZIMUTH_UNKNOWN == azi) {
        NMEA_SENTENCE_PRINTF(",");
      } else {
        NMEA_SENTENCE_PRINTF(",%03u", azi);
      }
#else
        NMEA_SENTENCE_PRINTF(",");
        NMEA_SENTENCE_PRINTF(",");
#endif

      NMEA_SENTENCE_PRINTF(",%02u", (u8)roundf(ch_meas[n]->cn0));

      j++; /* 4 sats per message no matter what */
    }
    NMEA_SENTENCE_DONE();
  }
}

/** Group tracked SVs by constellation and forward information to GSV printing
 * function.
 *
 * \note NMEA 0183 - Standard For Interfacing Marine Electronic Devices
 *       versions 2.30, 3.01 and 4.10 state following:
 *       If multiple GPS, GLONASS, Galileo, etc. satellites are in view, use
 *       separate GSV sentences with talker ID GP to show the GPS satellites in
 *       view and talker GL to show the GLONASS satellites in view and talker GA
 *       to show the Galileo satellites in view, etc. When more than ranging
 *       signal is used per satellite, also use separate GSV sentences with a
 *       Signal ID corresponding to the ranging signal. The GN identifier shall
 *       not be used with this sentence!
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of ch_measurement structs from SVs in track
 */
void nmea_gsv(u8 n_used, const channel_measurement_t *ch_meas) {
  if (0 == n_used || NULL == ch_meas) {
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF("$GPGSV,1,1,0");
    NMEA_SENTENCE_DONE();
    return;
  }

  /* Group by constellation */
  const channel_measurement_t *ch_meas_grouped[CONSTELLATION_COUNT][n_used];
  u8 n_gnss_used[CONSTELLATION_COUNT] = {0};

  for (u8 i = 0; i < n_used; i++) {
    constellation_t cons = code_to_constellation(ch_meas[i].sid.code);

    /* Special case, SBAS uses GP talker ID */
    if (CONSTELLATION_SBAS == cons) {
      cons = CONSTELLATION_GPS;
    }

    /* check if sat is already picked up from another code */
    bool in_array = false;
    for (u8 j = 0; j < n_gnss_used[cons]; j++) {
      if (ch_meas_grouped[cons][j]->sid.sat == ch_meas[i].sid.sat) {
        in_array = true;
        break;
      }
    }

    if (in_array) {
      /* SV already on the list */
      continue;
    }

    ch_meas_grouped[cons][n_gnss_used[cons]++] = &ch_meas[i];
  }

  /* Print grouped sentences */
  for (u8 cons = 0; cons < CONSTELLATION_COUNT; cons++) {
    if (0 == n_gnss_used[cons]) {
      continue;
    }
    nmea_gsv_print(n_gnss_used[cons], ch_meas_grouped[cons], cons);
  }
}

/** Calculate Course and Speed Over Ground values.
 *
 * \param[in]  sbp_vel_ned  pointer to sbp vel ned struct
 * \param[out] cog          true course over ground [deg]
 * \param[out] sog_knots    speed over ground [knots]
 * \param[out] sog_kph      speed over ground [kph]
 */
static void calc_cog_sog(const msg_vel_ned_t *sbp_vel_ned,
                         double *cog,
                         double *sog_knots,
                         double *sog_kph) {
  double vel_north_ms = MM2M(sbp_vel_ned->n);
  double vel_east_ms = MM2M(sbp_vel_ned->e);

  *cog = R2D * atan2(vel_east_ms, vel_north_ms);

  /* Convert negative values to positive */
  if (*cog < 0.0) {
    *cog += FULL_CIRCLE_DEG;
  }

  /* Rounding to specified accuracy */
  *cog = roundf(*cog * NMEA_COG_FRAC_DIVISOR) / NMEA_COG_FRAC_DIVISOR;

  /* Avoid having duplicate values for same point (0 and 360) */
  if (fabs(FULL_CIRCLE_DEG - *cog) < 1 / NMEA_COG_FRAC_DIVISOR) {
    *cog = 0;
  }

  *sog_knots = MS2KNOTS(vel_north_ms, vel_east_ms, 0);
  *sog_kph = MS2KMHR(vel_north_ms, vel_east_ms, 0);
}

/** Assemble an NMEA GPRMC message and send it out NMEA USARTs.
 * NMEA RMC contains Recommended Minimum Specific GNSS Data.
 *
 * \param sbp_pos_llh  pointer to sbp pos llh struct
 * \param sbp_vel_ned  pointer to sbp vel ned struct
 * \param sbp_msg_time Pointer to sbp gps time struct
 * \param utc_time     Pointer to UTC time
 */
void nmea_gprmc(const msg_pos_llh_t *sbp_pos_llh,
                const msg_vel_ned_t *sbp_vel_ned,
                const msg_gps_time_t *sbp_msg_time,
                const utc_tm *utc_time) {
  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);
  char status = get_nmea_status(sbp_pos_llh->flags);

  double cog, sog_knots, sog_kph;
  calc_cog_sog(sbp_vel_ned, &cog, &sog_knots, &sog_kph);

  NMEA_SENTENCE_START(140);
  NMEA_SENTENCE_PRINTF("$GPRMC,"); /* Command */

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, false, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF("%c,", /* Status */
                       status);

  if ((sbp_pos_llh->flags & POSITION_MODE_MASK) != NO_POSITION) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,", /* Lat/Lon */
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,"); /* Lat/Lon */
  }

  if ((sbp_pos_llh->flags & VELOCITY_MODE_MASK) != NO_VELOCITY) {
    NMEA_SENTENCE_PRINTF("%.2f,", sog_knots); /* Speed */
    if (NMEA_COG_STATIC_LIMIT_KNOTS < sog_knots) {
      NMEA_SENTENCE_PRINTF("%.*f,", NMEA_COG_DECIMALS, cog); /* Course */
    } else {
      NMEA_SENTENCE_PRINTF(","); /* Course */
    }
  } else {
    NMEA_SENTENCE_PRINTF(",,"); /* Speed, Course */
  }

  char date[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, false, true, true, utc_time, date, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", date);

  NMEA_SENTENCE_PRINTF(
      ",,"  /* Magnetic Variation */
      "%c", /* Mode Indicator */
      mode);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPVTG message and send it out NMEA USARTs.
 * NMEA VTG contains Course Over Ground & Ground Speed.
 *
 * \param sbp_vel_ned Pointer to sbp vel ned struct.
 */
void nmea_gpvtg(const msg_vel_ned_t *sbp_vel_ned,
                const msg_pos_llh_t *sbp_pos_llh) {
  double cog, sog_knots, sog_kph;
  calc_cog_sog(sbp_vel_ned, &cog, &sog_knots, &sog_kph);

  /* Position indicator is used based upon spec
     "Positioning system mode indicator" means we should
     see the same mode for pos and velocity messages
     in a particular epoch */

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPVTG,"); /* Command */

  bool is_moving = (sbp_pos_llh->flags & VELOCITY_MODE_MASK) != NO_VELOCITY;

  if (is_moving && NMEA_COG_STATIC_LIMIT_KNOTS < sog_knots) {
    NMEA_SENTENCE_PRINTF("%.*f,T,", NMEA_COG_DECIMALS, cog); /* Course */
  } else {
    NMEA_SENTENCE_PRINTF(",T,"); /* Course */
  }

  NMEA_SENTENCE_PRINTF(",M,"); /* Magnetic Course (omitted) */

  if (is_moving) {
    /* Speed (knots, km/hr) */
    NMEA_SENTENCE_PRINTF("%.2f,N,%.2f,K,", sog_knots, sog_kph);
  } else {
    /* Speed (knots, km/hr) */
    NMEA_SENTENCE_PRINTF(",N,,K,");
  }

  /* Mode (note this is position mode not velocity mode)*/
  NMEA_SENTENCE_PRINTF("%c", mode);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPHDT message and send it out NMEA USARTs.
 * NMEA HDT contains Heading.
 *
 */
void nmea_gphdt(const msg_baseline_heading_t *sbp_baseline_heading) {
  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF("$GPHDT,"); /* Command */
  if ((POSITION_MODE_MASK & sbp_baseline_heading->flags) == FIXED_POSITION) {
    NMEA_SENTENCE_PRINTF(
        "%.1f,T",
        (float)sbp_baseline_heading->heading /
            MSG_HEADING_SCALE_FACTOR); /* Heading only valid when fixed */
  } else {
    NMEA_SENTENCE_PRINTF(",T"); /* Heading only valid when fixed */
  }
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPGLL message and send it out NMEA USARTs.
 * NMEA GLL contains Geographic Position Latitude/Longitude.
 *
 * \param sbp_pos_llh  Pointer to sbp pos llh struct.
 * \param sbp_msg_time Pointer to sbp gps time struct.
 * \param utc_time     Pointer to UTC time.
 */
void nmea_gpgll(const msg_pos_llh_t *sbp_pos_llh,
                const msg_gps_time_t *sbp_msg_time,
                const utc_tm *utc_time) {
  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char status = get_nmea_status(sbp_pos_llh->flags);
  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGLL,"); /* Command */

  if ((sbp_pos_llh->flags & POSITION_MODE_MASK) != NO_POSITION) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,", /* Lat/Lon */
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,"); /* Lat/Lon */
  }

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, false, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF("%c,%c", /* Status, Mode */
                       status,
                       mode);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPZDA message and send it out NMEA USARTs.
 * NMEA ZDA contains UTC Time and Date.
 *
 * \param sbp_msg_time Pointer to the current SBP GPS Time.
 * \param utc_time     Pointer to UTC time
 */
void nmea_gpzda(const msg_gps_time_t *sbp_msg_time, const utc_tm *utc_time) {
  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF("$GPZDA,"); /* Command */

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, true, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF(","); /* Time zone */
  NMEA_SENTENCE_DONE();

} /* nmea_gpzda() */

/** Generate and send periodic GPGSV and GLGSV.
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of channel_measurement structs from tracked SVs
 */
void nmea_send_gsv(u8 n_used, const channel_measurement_t *ch_meas) {
  (void)n_used;
  (void)ch_meas;
  //DO_EVERY(gpgsv_msg_rate, nmea_gsv(n_used, ch_meas));
}

bool send_nmea(u32 rate, u32 gps_tow_ms) {
  if (rate == 0) {
    return false;
  }

#if 0
  /* If the modulo of latest gps time estimate time with configured
  * output period is less than 1/2 the solution period we should send the NMEA
  * message.
  * This way, we still send no_fix messages when receiver clock is drifting. */
  u32 soln_period_ms = (u32)(1.0 / soln_freq_setting * 1e3);
  u32 output_period_ms = (u32)soln_period_ms * rate;
  if ((gps_tow_ms % output_period_ms) < (soln_period_ms / 2)) {
    return true;
  }
#else
  (void)gps_tow_ms;
#endif
  return false;
}

#if 0
/** Generate and send periodic NMEA GPRMC, GPGLL, GPVTG, GPZDA and GPGSA.
 * (but not GPGGA) messages.
 *
 * Called from solution thread.
 *
 * \param sbp_pos_llh  Pointer to sbp pos llh struct.
 * \param sbp_vel_ned  Pointer to sbp vel ned.
 * \param sbp_dops     Pointer to sbp dops.
 * \param sbp_msg_time Pointer to sbp msg time.
 * \param propagation_time time of base observation propagation
 * \param sender_id    NMEA sender id
 * \param utc_params   Pointer to UTC parameters
 * \param n_meas       nav_meas len
 * \param nav_meas     Valid navigation measurements
 */
void nmea_send_msgs(const msg_pos_llh_t *sbp_pos_llh,
                    const msg_vel_ned_t *sbp_vel_ned,
                    const msg_dops_t *sbp_dops,
                    const msg_gps_time_t *sbp_msg_time,
                    double propagation_time,
                    u8 sender_id,
                    const utc_params_t *utc_params,
                    const msg_baseline_heading_t *sbp_baseline_heading,
                    u8 n_meas,
                    const navigation_measurement_t nav_meas[]) {
  utc_tm utc_time;
  static bool first_fix = false;

  piksi_systime_t piksi_system_time;
  u64 piksi_time_ms;
  piksi_systime_get(&piksi_system_time);
  /* piksi_time_ms will be used to decimate output before the first PVT
   * solution */
  piksi_time_ms = piksi_systime_to_ms(&piksi_system_time);

  /* prepare utc_tm structure with time rounded to NMEA precision */
  if ((sbp_msg_time->flags & TIME_SOURCE_MASK) != NO_TIME) {
    first_fix = true;
    gps_time_t t = {
        .wn = sbp_msg_time->wn,
        .tow = 1e-3 * sbp_msg_time->tow + 1e-9 * sbp_msg_time->ns_residual};
    gps2utc(&t, &utc_time, utc_params);
    u16 second_frac = roundf(utc_time.second_frac * NMEA_UTC_S_FRAC_DIVISOR);
    if (second_frac == NMEA_UTC_S_FRAC_DIVISOR) {
      /* the UTC timestamp rounds up to next second, so recompute the UTC time
       * structure to normalize it and to handle leap second correctly */
      double dt = 1.0 - utc_time.second_frac;
      /* round up to the next representable floating point number */
      t.tow = nextafter(t.tow + dt, INFINITY);
      normalize_gps_time(&t);
      gps2utc(&t, &utc_time, utc_params);
      /* if the fractional part of the resulting new time stamp is not zero
       * then we computed wrong... */
      second_frac = roundf(utc_time.second_frac * NMEA_UTC_S_FRAC_DIVISOR);
      assert(second_frac == 0);
    }
  }

  if (first_fix) {
    piksi_time_ms = sbp_msg_time->tow;
  }

  if (sbp_pos_llh && sbp_msg_time && sbp_dops) {
    if (send_nmea(gpgga_msg_rate, piksi_time_ms)) {
      nmea_gpgga(sbp_pos_llh,
                 sbp_msg_time,
                 &utc_time,
                 sbp_dops,
                 propagation_time,
                 sender_id);
    }
  }
  if (sbp_baseline_heading && send_heading && sbp_msg_time) {
    if (send_nmea(gphdt_msg_rate, piksi_time_ms)) {
      nmea_gphdt(sbp_baseline_heading);
    }
  }
  if (sbp_vel_ned && sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gprmc_msg_rate, piksi_time_ms)) {
      nmea_gprmc(sbp_pos_llh, sbp_vel_ned, sbp_msg_time, &utc_time);
    }
  }
  if (sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gpgll_msg_rate, piksi_time_ms)) {
      nmea_gpgll(sbp_pos_llh, sbp_msg_time, &utc_time);
    }
  }
  if (sbp_vel_ned && sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gpvtg_msg_rate, piksi_time_ms)) {
      nmea_gpvtg(sbp_vel_ned, sbp_pos_llh);
    }
  }
  if (sbp_msg_time) {
    if (send_nmea(gpzda_msg_rate, piksi_time_ms)) {
      nmea_gpzda(sbp_msg_time, &utc_time);
    }
  }
  if (sbp_dops && sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gsa_msg_rate, piksi_time_ms)) {
      nmea_gsa(sbp_pos_llh, sbp_dops, n_meas, nav_meas);
    }
  }
}
#endif

/** Convert the SBP status flag into NMEA Status field.
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_status(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case NO_POSITION:
      return 'V';
    case SPP_POSITION: /* autonomous mode */
    case DGNSS_POSITION:
    case SBAS_POSITION:
    case FLOAT_POSITION:
    case FIXED_POSITION:
      return 'A';
    default:
      assert(!"Unsupported position type indicator");
      return 'V';
  }
}

/** Convert the SBP status flag into NMEA Mode Indicator field:
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_mode_indicator(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case NO_POSITION:
      return 'N';
    case SPP_POSITION: /* autonomous mode */
      return 'A';
    case DGNSS_POSITION: /* differential mode */
    case SBAS_POSITION:
    case FLOAT_POSITION:
    case FIXED_POSITION:
      return 'D';
    default:
      assert(!"Unsupported position type indicator");
      return 'N';
  }
}

/** Convert the SBP status flag into NMEA Quality Indicator field:
 * Ref: NMEA-0183 version 2.30 pp.30
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
u8 get_nmea_quality_indicator(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case NO_POSITION:
      return NMEA_GGA_QI_INVALID;
    case SPP_POSITION:
      return NMEA_GGA_QI_GPS;
    case DGNSS_POSITION:
    case SBAS_POSITION:
      return NMEA_GGA_QI_DGPS;
    case FLOAT_POSITION:
      return NMEA_GGA_QI_FLOAT;
    case FIXED_POSITION:
      return NMEA_GGA_QI_RTK;
    default:
      assert(!"Unsupported position type indicator");
      return NMEA_GGA_QI_INVALID;
  }
}

/** \} */

/** \} */
