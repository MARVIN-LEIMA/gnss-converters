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

#include <assert.h>

#include <libswiftnav/signal.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/shm.h>

/** Decode SHI1
 *  Refer to libswiftnav/shm.h for details.
 *
 * \param sf1w3 word number 3 of subframe number 1
 * \param shi1 pointer to an SHI1 variable to fill in
 */
void shm_gps_decode_shi1(u32 sf1w3, u8* shi1)
{
  *shi1 = (sf1w3 >> 8) & 0x3F;
}

/** The six-bit health indication given by bits 17 through 22 of word three
 * refers to the transmitting SV. The MSB shall indicate a summary of the
 * health of the NAV data, where
 *   0 = all NAV data are OK,
 *   1 = some or all NAV data are bad.
 *
 * http://www.gps.gov/technical/icwg/IS-GPS-200H.pdf
 * 20.3.3.3.1.4 SV Health.
 *
 * \param health_bits SV health bits
 * \return true if the satellite health summary is OK.
 *         false otherwise.
 */
static bool gps_nav_data_health_summary(u8 health_bits) {
  u8 summary = (health_bits >> 5) & 0x1;

  return (0 == summary);
}

typedef enum gps_signal_component_health {
  ALL_SIGNALS_OK = 0,
  ALL_SIGNALS_WEAK,
  ALL_SIGNALS_DEAD,
  ALL_SIGNALS_NO_DATA,
  L1_P_SIGNAL_WEAK,
  L1_P_SIGNAL_DEAD,
  L1_P_SIGNAL_NO_DATA,
  L2_P_SIGNAL_WEAK,
  L2_P_SIGNAL_DEAD,
  L2_P_SIGNAL_NO_DATA,
  L1_C_SIGNAL_WEAK,
  L1_C_SIGNAL_DEAD,
  L1_C_SIGNAL_NO_DATA,
  L2_C_SIGNAL_WEAK,
  L2_C_SIGNAL_DEAD,
  L2_C_SIGNAL_NO_DATA,
  L1_L2_P_SIGNAL_WEAK,
  L1_L2_P_SIGNAL_DEAD,
  L1_L2_P_SIGNAL_NO_DATA,
  L1_L2_C_SIGNAL_WEAK,
  L1_L2_C_SIGNAL_DEAD,
  L1_L2_C_SIGNAL_NO_DATA,
  L1_SIGNAL_WEAK,
  L1_SIGNAL_DEAD,
  L1_SIGNAL_NO_DATA,
  L2_SIGNAL_WEAK,
  L2_SIGNAL_DEAD,
  L2_SIGNAL_NO_DATA,
  SV_TEMPORARILY_OUT,
  SV_WILL_BE_TEMPORARILY_OUT,
  ONLY_URA_VALID,
  MULTIPLE_PROBLEMS
} gps_signal_component_health_t;

/** Check GPS NAV data health summary and signal component status.
 *
 *  From IS-GPS-200H:
 *  The six-bit words provide a one-bit summary of the NAV data's health status
 *  in the MSB position in accordance with paragraph 20.3.3.3.1.4. The five LSBs
 *  of both the eight-bit and the six-bit words provide the health status of the
 *  SV's signal components in accordance with the code given in Table 20-VIII.
 *
 * \param health_bits SV health bits. Refer to gps_nav_data_health_summary()
 *        documentation above for details.
 * \param code code for which health should be analyzed
 *
 * \return true if current signal is OK.
 *         false otherwise.
 */
bool check_6bit_health_word(const u8 health_bits, const code_t code) {
  /* Check NAV data health summary */
  if (((CODE_GPS_L1CA == code) || (CODE_GPS_L1P == code)) &&
      !gps_nav_data_health_summary(health_bits)) {
    return 0;
  }

  const u8 b = health_bits & 0x1f;

  /* Check general issues */
  if (b == ALL_SIGNALS_WEAK || b == ALL_SIGNALS_DEAD ||
      b == ALL_SIGNALS_NO_DATA || b == SV_TEMPORARILY_OUT ||
      b == SV_WILL_BE_TEMPORARILY_OUT || b == ONLY_URA_VALID ||
      b == MULTIPLE_PROBLEMS) {
    return 0;
  }

  /* Check code specific issues */
  switch (code) {
    case CODE_GPS_L1CA:
      if (b == L1_C_SIGNAL_WEAK || b == L1_C_SIGNAL_DEAD ||
          b == L1_C_SIGNAL_NO_DATA || b == L1_L2_C_SIGNAL_WEAK ||
          b == L1_L2_C_SIGNAL_DEAD || b == L1_L2_C_SIGNAL_NO_DATA ||
          b == L1_SIGNAL_WEAK || b == L1_SIGNAL_DEAD ||
          b == L1_SIGNAL_NO_DATA) {
        return 0;
      }
      return 1;

    case CODE_GPS_L2CM:
      if (b == L2_C_SIGNAL_WEAK || b == L2_C_SIGNAL_DEAD ||
          b == L2_C_SIGNAL_NO_DATA || b == L1_L2_C_SIGNAL_WEAK ||
          b == L1_L2_C_SIGNAL_DEAD || b == L1_L2_C_SIGNAL_NO_DATA ||
          b == L2_SIGNAL_WEAK || b == L2_SIGNAL_DEAD ||
          b == L2_SIGNAL_NO_DATA) {
        return 0;
      }
      return 1;

    case CODE_GPS_L1P:
      if (b == L1_P_SIGNAL_WEAK || b == L1_P_SIGNAL_DEAD ||
          b == L1_P_SIGNAL_NO_DATA || b == L1_L2_P_SIGNAL_WEAK ||
          b == L1_L2_P_SIGNAL_DEAD || b == L1_L2_P_SIGNAL_NO_DATA ||
          b == L1_SIGNAL_WEAK || b == L1_SIGNAL_DEAD ||
          b == L1_SIGNAL_NO_DATA) {
        return 0;
      }
      return 1;

    case CODE_GPS_L2P:
      if (b == L2_P_SIGNAL_WEAK || b == L2_P_SIGNAL_DEAD ||
          b == L2_P_SIGNAL_NO_DATA || b == L1_L2_P_SIGNAL_WEAK ||
          b == L1_L2_P_SIGNAL_DEAD || b == L1_L2_P_SIGNAL_NO_DATA ||
          b == L2_SIGNAL_WEAK || b == L2_SIGNAL_DEAD ||
          b == L2_SIGNAL_NO_DATA) {
        return 0;
      }
      return 1;

    case CODE_INVALID:
    case CODE_COUNT:
    case CODE_GPS_L2CL:
    case CODE_SBAS_L1CA:
    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    default:
      assert(!"Unsupported code");
      return 0;
  }
}
