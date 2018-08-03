

#include "rtcm3_sbp_internal.h"
#include <math.h>

#define power2_55 pow(2,-55) /* 2^-31 */
#define power2_43 pow(2,-43) /* 2^-31 */
#define power2_33 pow(2,-33) /* 2^-31 */
#define power2_31 pow(2,-31) /* 2^-31 */
#define power2_29 pow(2,-29) /* 2^-31 */
#define power2_19 pow(2,-19) /* 2^-31 */
#define power2_5 pow(2,-5) /* 2^5) */
#define SC2RAD 3.1415926535898     /* semi-circle to radian (IS-GPS) */

void rtcm3_gps_eph_to_sbp(rtcm_msg_eph *msg_eph, msg_ephemeris_gps_t *sbp_gps_eph) {
  sbp_gps_eph->common.toe.wn = msg_eph->wn;
  sbp_gps_eph->common.toe.tow = msg_eph->toe;
  sbp_gps_eph->common.sid.sat = msg_eph->sat_id;
  sbp_gps_eph->common.sid.code = CODE_GPS_L1CA;
  sbp_gps_eph->common.ura = msg_eph->ura;
  sbp_gps_eph->common.fit_interval = msg_eph->fit_interval;
  sbp_gps_eph->common.valid = msg_eph->valid;
  sbp_gps_eph->common.health_bits = msg_eph->health_bits;

  sbp_gps_eph->tgd = msg_eph->kepler.tgd_gps_s * power2_31;

  sbp_gps_eph->c_rs = msg_eph->kepler.crs * power2_5;
  sbp_gps_eph->c_rc = msg_eph->kepler.crc * power2_5;
  sbp_gps_eph->c_uc = msg_eph->kepler.cuc * power2_29;
  sbp_gps_eph->c_us = msg_eph->kepler.cus * power2_29;
  sbp_gps_eph->c_ic = msg_eph->kepler.cic * power2_29;
  sbp_gps_eph->c_is = msg_eph->kepler.cis * power2_29;

  sbp_gps_eph->dn = msg_eph->kepler.dn * power2_43 * SC2RAD;
  sbp_gps_eph->m0 = msg_eph->kepler.m0 * power2_31 * SC2RAD;
  sbp_gps_eph->ecc = msg_eph->kepler.ecc * power2_33;
  sbp_gps_eph->sqrta = msg_eph->kepler.sqrta * power2_19;
  sbp_gps_eph->omega0 = msg_eph->kepler.omega0 * power2_31 * SC2RAD;
  sbp_gps_eph->omegadot = msg_eph->kepler.omegadot * power2_43 * SC2RAD;
  sbp_gps_eph->w = msg_eph->kepler.w * power2_31 * SC2RAD;
  sbp_gps_eph->inc = msg_eph->kepler.inc * power2_31 * SC2RAD;
  sbp_gps_eph->inc_dot = msg_eph->kepler.inc_dot * power2_43 * SC2RAD;

  sbp_gps_eph->af0 = msg_eph->kepler.af0 * power2_55;
  sbp_gps_eph->af1 = msg_eph->kepler.af1 * power2_43;
  sbp_gps_eph->af2 = msg_eph->kepler.af2 * power2_31;

  sbp_gps_eph->iode = msg_eph->kepler.iode;
  sbp_gps_eph->iodc = msg_eph->kepler.iodc;

  sbp_gps_eph->toc.wn = msg_eph->wn;
  sbp_gps_eph->toc.tow = msg_eph->kepler.toc;

}
void rtcm3_glo_eph_to_sbp(rtcm_msg_eph *msg_eph, msg_ephemeris_glo_t *sbp_glo_eph) {
  sbp_glo_eph->common.toe.wn = msg_eph->wn;
}
