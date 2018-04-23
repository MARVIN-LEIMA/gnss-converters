/*
 * Copyright (C) 2010, 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *          Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include <libswiftnav/logging.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/bits.h>
#include <libswiftnav/shm.h>

/** \defgroup ephemeris Ephemeris
 * Functions and calculations related to the GPS ephemeris.
 * \{ */

/* maximum step length in seconds for Runge-Kutta aglorithm */
#define GLO_MAX_STEP_LENGTH 30

/* maximum steps for Runge-Kutta algorithm,
 * based on modeling https://github.com/swift-nav/exafore_planning/issues/681 */
#define GLO_MAX_STEP_NUM 30

u32 decode_fit_interval(u8 fit_interval_flag, u16 iodc);

/**
 * Helper to sign extend 14-bit value
 *
 * \param[in] arg Unsigned integer
 *
 * \return Sign-extended integer
 */
static inline s32 sign_extend14(u32 arg)
{
  return BITS_SIGN_EXTEND_32(14, arg);
}

/**
 * Helper to sign extend 22-bit value
 *
 * \param[in] arg Unsigned integer
 *
 * \return Sign-extended integer
 */
static inline s32 sign_extend22(u32 arg)
{
  return BITS_SIGN_EXTEND_32(22, arg);
}

/**
 * Helper to sign extend 24-bit value

 * \param[in] arg Unsigned integer
 *
 * \return Sign-extended integer
 */
static inline s32 sign_extend24(u32 arg)
{
  return BITS_SIGN_EXTEND_32(24, arg);
}

/** Calculate satellite position, velocity and clock offset from SBAS ephemeris.
 *
 * References:
 *   -# WAAS Specification FAA-E-2892b 4.4.11
 *
 * \param e Pointer to an ephemeris structure for the satellite of interest
 * \param t GPS time at which to calculate the satellite state
 * \param pos Array into which to write calculated satellite position [m]
 * \param vel Array into which to write calculated satellite velocity [m/s]
 * \param clock_err Pointer to where to store the calculated satellite clock
 *                  error [s]
 * \param clock_rate_err Pointer to where to store the calculated satellite
 *                       clock error [s/s]
 *
 * \return  0 on success,
 *         -1 if ephemeris is not valid or too old
 */
static s8 calc_sat_state_xyz(const ephemeris_t *e, const gps_time_t *t,
                             double pos[3], double vel[3], double acc[3],
                             double *clock_err, double *clock_rate_err,
                             u16 *iodc, u8 *iode)
{
  /* TODO should t be in GPS or SBAS time? */
  /* TODO what is the SBAS valid ttime interval? */

  const ephemeris_xyz_t *ex = &e->xyz;

  double dt = gpsdifftime(t, &e->toe);

  vel[0] = ex->vel[0] + ex->acc[0] * dt;
  vel[1] = ex->vel[1] + ex->acc[1] * dt;
  vel[2] = ex->vel[2] + ex->acc[2] * dt;

  double dt2 = dt * dt;

  pos[0] = ex->pos[0] + ex->vel[0] * dt +
           0.5 * ex->acc[0] * dt2;
  pos[1] = ex->pos[1] + ex->vel[1] * dt +
           0.5 * ex->acc[1] * dt2;
  pos[2] = ex->pos[2] + ex->vel[2] * dt +
           0.5 * ex->acc[2] * dt2;

  acc[0] = ex->acc[0];
  acc[1] = ex->acc[1];
  acc[2] = ex->acc[2];

  *clock_err = ex->a_gf0;
  *clock_rate_err = ex->a_gf1;

  // SBAS doesn't have an IODE so just set to 0.
  *iodc = 0;
  *iode = 0;

  return 0;
}

/** Re-calculation of the acceleration in ECEF using
 *  a position and velocity (ECEF) and acceleration term (ECI)
 *
 *  ICD 5.1: A.3.1.2, with corrections from RTCM 3.2 p.186
 *
 * \param vel_acc Pointer to concatenation of velocities and accelerations (ECEF)
 * \param pos Pointer to position input array (ECEF)
 * \param vel Pointer to velocity input array (ECEF)
 * \param acc Pointer to acceleration input array (ECI)
 */
static void calc_ecef_vel_acc(double vel_acc[6],
                              const double pos[3],
                              const double vel[3],
                              const double acc[3])
{

  double r = sqrt(pos[0]*pos[0] +
                  pos[1]*pos[1] +
                  pos[2]*pos[2]);

  double m_r3 = GLO_GM / (r * r * r);
  double inv_r2 = 1 / (r * r);

  double g_term = 3.0/2.0 * GLO_J02 * m_r3 *
                  GLO_A_E * GLO_A_E * inv_r2;

  double lg_term = (1.0 - 5.0 * pos[2] * pos[2] * inv_r2);

  double omega_sqr = GLO_OMEGAE_DOT * GLO_OMEGAE_DOT;

  vel_acc[0] = vel[0];
  vel_acc[1] = vel[1];
  vel_acc[2] = vel[2];

  vel_acc[3] = -m_r3 * pos[0]
            -g_term * pos[0] * lg_term
            + omega_sqr * pos[0]
            + 2.0 * GLO_OMEGAE_DOT * vel[1]
            + acc[0];

  vel_acc[4] = -m_r3 * pos[1]
            -g_term * pos[1] * lg_term
            + omega_sqr * pos[1]
            - 2.0 * GLO_OMEGAE_DOT * vel[0]
            + acc[1];

  vel_acc[5] = -m_r3 * pos[2]
            -g_term * pos[2] * (2.0 + lg_term)
            + acc[2];
}

/** Calculate satellite position, velocity and clock offset from GLO ephemeris.
 *
 * \param e Pointer to an ephemeris structure for the satellite of interest
 * \param t time at which to calculate the satellite state
 * \param pos Array into which to write calculated satellite position [m]
 * \param vel Array into which to write calculated satellite velocity [m/s]
 * \param clock_err Pointer to where to store the calculated satellite clock
 *                  error [s]
 * \param clock_rate_err Pointer to where to store the calculated satellite
 *                       clock error [s/s]
 *
 */
static s8 calc_sat_state_glo(const ephemeris_t *e, const gps_time_t *t,
                             double pos[3], double vel[3], double acc[3],
                             double *clock_err, double *clock_rate_err,
                             u16 *iodc, u8 *iode)
{
  assert(e != NULL);
  assert(t != NULL);
  assert(pos != NULL);
  assert(vel != NULL);
  assert(acc != NULL);
  assert(clock_err != NULL);
  assert(clock_rate_err != NULL);

  /* NOTE: toe should be in GPS time as well */
  double dt = gpsdifftime(t, &e->toe);

  u32 num_steps = ceil(fabs(dt) / GLO_MAX_STEP_LENGTH);
  num_steps = MIN(num_steps, GLO_MAX_STEP_NUM);

  double ecef_vel_acc[6];
  if (num_steps) {
    double h = dt / num_steps;

    double y[6];

    calc_ecef_vel_acc(ecef_vel_acc, e->glo.pos, e->glo.vel, e->glo.acc);
    memcpy(&y[0], e->glo.pos, sizeof(double) * 3);
    memcpy(&y[3], e->glo.vel, sizeof(double) * 3);

    /* Runge-Kutta integration algorithm */
    for (u32 i = 0; i < num_steps; i++) {

      double k1[6], k2[6], k3[6], k4[6], y_tmp[6];
      u8 j;

      memcpy(k1, ecef_vel_acc, sizeof(k1));

      for (j = 0; j < 6; j++)
        y_tmp[j] = y[j] + h/2 * k1[j];

      calc_ecef_vel_acc(k2, &y_tmp[0], &y_tmp[3], e->glo.acc);

      for (j = 0; j < 6; j++)
        y_tmp[j] = y[j] + h/2 * k2[j];

      calc_ecef_vel_acc(k3, &y_tmp[0], &y_tmp[3], e->glo.acc);

      for (j = 0; j < 6; j++)
        y_tmp[j] = y[j] + h * k3[j];

      calc_ecef_vel_acc(k4, &y_tmp[0], &y_tmp[3], e->glo.acc);

      for (j = 0; j < 6; j++)
        y[j] += h/6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);

      calc_ecef_vel_acc(ecef_vel_acc, &y[0], &y[3], e->glo.acc);
    }
    memcpy(pos, &y[0], sizeof(double) * 3);
    memcpy(vel, &y[3], sizeof(double) * 3);
  } else {
    memcpy(pos, e->glo.pos, sizeof(double) * 3);
    memcpy(vel, e->glo.vel, sizeof(double) * 3);
  }
  // Here we compute the final acceleration (ECEF).
  calc_ecef_vel_acc(ecef_vel_acc, pos, vel, e->glo.acc);
  memcpy(acc, &ecef_vel_acc[3], sizeof(double) * 3);

  *clock_err = -e->glo.tau + e->glo.gamma * dt;
  *clock_err -= get_tgd_correction(e,&e->sid);

  *clock_rate_err = e->glo.gamma;
  *iodc = e->glo.iod;
  *iode = e->glo.iod;

  return 0;
}

/** Calculate satellite position, velocity and clock offset from GPS ephemeris.
 *
 * References:
 *   -# IS-GPS-200D, Section 20.3.3.3.3.1 and Table 20-IV
 *   -http://math.tut.fi/posgroup/korvenoja_piche_ion2000a.pdf which
 *    was used to implement the acceleration terms, note however there are
 *    several typos.  In particular in the equation for z_s'' the inc ** 2
 *    term should be inc' ** 2.  And for x_s'' the omega' x_p term should
 *    be omega' x_p'.  Each of these can be confirmed by checking units.
 *
 * \param e Pointer to an ephemeris structure for the satellite of interest
 * \param t GPS time at which to calculate the satellite state
 * \param pos Array into which to write calculated satellite position [m]
 * \param vel Array into which to write calculated satellite velocity [m/s]
 * \param clock_err Pointer to where to store the calculated satellite clock
 *                  error [s]
 * \param clock_rate_err Pointer to where to store the calculated satellite
 *                       clock error [s/s]
 *
 * \return  0 on success,
 *         -1 if ephemeris is not valid or too old
 */
static s8 calc_sat_state_kepler(const ephemeris_t *e,
                                const gps_time_t *t,
                                double pos[3], double vel[3], double acc[3],
                                double *clock_err, double *clock_rate_err,
                                u16 *iodc, u8 *iode)
{
  const ephemeris_kepler_t *k = &e->kepler;

  /* Calculate satellite clock terms */

  /* Seconds from clock data reference time (toc) */
  double dt = gpsdifftime(t, &k->toc);

  /* According to the GPS ICD the satellite clock is reported
   * in iono free form, which means that the clock errors for
   * L1 and L2 need to take the group delay into account */
  *clock_err = k->af0 + dt * (k->af1 + dt * k->af2) - get_tgd_correction(e,&e->sid);
  *clock_rate_err = k->af1 + 2.0 * dt * k->af2;
  *iodc = k->iodc;

  /* Seconds from the time from ephemeris reference epoch (toe) */
  dt = gpsdifftime(t, &e->toe);

  /* Calculate position per IS-GPS-200D p 97 Table 20-IV */

  /* Semi-major axis in meters. */
  double a = k->sqrta * k->sqrta;
  /* Corrected mean motion in radians/sec. */
  double ma_dot = sqrt(GPS_GM / (a * a * a)) + k->dn;
  /* Corrected mean anomaly in radians. */
  double ma = k->m0 + ma_dot * dt;

  /* Iteratively solve for the Eccentric Anomaly
   * (from Keith Alter and David Johnston) */
  double ea = ma; /* Starting value for E. */
  double ea_old;
  double temp;
  double ecc = k->ecc;
  u8 count = 0;

  /* TODO: Implement convergence test using integer difference of doubles,
   * http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm */
  do {
    ea_old = ea;
    temp = 1.0 - ecc * cos(ea_old);
    ea = ea + (ma - ea_old + ecc * sin(ea_old)) / temp;
    count++;
    if (count > 5)
      break;
  } while (fabs(ea - ea_old) > 1.0E-14);

  double ea_dot = ma_dot / temp;
  double ea_acc = ea_dot * ea_dot * ecc * sin(ea) / temp;

  /* Begin calc for True Anomaly and Argument of Latitude */
  double temp2 = sqrt(1.0 - ecc * ecc);
  /* Argument of Latitude = True Anomaly + Argument of Perigee. */
  double al = atan2(temp2 * sin(ea), cos(ea) - ecc) + k->w;
  double al_dot = temp2 * ea_dot / temp;
  double al_acc = 2 * al_dot * ea_acc / ea_dot;
  double al_dot_sqr = al_dot * al_dot;

  /* These values are used all over the place, only do them once. */
  double sin2al = sin(2.0 * al);
  double cos2al = cos(2.0 * al);

  /* Calculate the argument of latitude correction */
  double dal = k->cus * sin2al + k->cuc * cos2al;
  double dal_dot = 2 * al_dot * (k->cus * cos2al - k->cuc * sin2al);
  double dal_acc = - 4 * al_dot_sqr * dal + al_acc / al_dot * dal_dot;

  /* Calculate corrected argument of latitude based on position. */
  double cal = al + dal;
  double cal_dot = al_dot + dal_dot;
  double cal_acc = al_acc + dal_acc;

  /* Calculate the radius correction */
  double dr = (k->crs * sin2al + k->crc * cos2al);
  double dr_dot = 2 * al_dot * (k->crs * cos2al - k->crc * sin2al);
  double dr_acc = 4 * al_dot_sqr * dr + al_acc / al_dot * dr_dot;

  /* Calculate corrected radius based on argument of latitude. */
  double r = a * temp + k->crc * cos2al + k->crs * sin2al;
  double r_dot = a * ecc * sin(ea) * ea_dot
                 + 2.0 * al_dot * (k->crs * cos2al
                                   - k->crc * sin2al);
  double r_acc = a * ecc * ea_dot * ea_dot * cos(ea) + a * ecc * ea_acc * sin(ea) + dr_acc;

  /* Relativistic correction term to satellite clock using x.v = r . r_dot. */
  double einstein = -2.0 * r * r_dot / GPS_C / GPS_C;
  *clock_err += einstein;

  /* Calculate the inclination correction */
  double dinc = (k->cis * sin2al + k->cic * cos2al);
  double dinc_dot = 2 * al_dot * (k->cis * cos2al - k->cic * sin2al);
  double dinc_acc = - 4 * al_dot_sqr * dinc + al_acc / al_dot * dinc_dot;

  /* Calculate inclination based on argument of latitude. */
  double inc = k->inc + k->inc_dot * dt + k->cic * cos2al
               + k->cis * sin2al;
  double inc_dot = k->inc_dot
                   + 2.0 * al_dot * (k->cis * cos2al
                                     - k->cic * sin2al);
  double inc_acc = dinc_acc;

  /* Calculate position and velocity in orbital plane. */
  double x = r * cos(cal);
  double y = r * sin(cal);
  double x_dot = r_dot * cos(cal) - y * cal_dot;
  double y_dot = r_dot * sin(cal) + x * cal_dot;
  double cal_dot_sqr = cal_dot * cal_dot;
  double x_acc = -cal_dot_sqr * x - cal_acc * y - 2 * cal_dot * r_dot * sin(cal) + r_acc * cos(cal);
  double y_acc = -cal_dot_sqr * y + cal_acc * x + 2 * cal_dot * r_dot * cos(cal) + r_acc * sin(cal);

  /* Corrected longitude of ascenting node. */
  double om_dot = k->omegadot - GPS_OMEGAE_DOT;
  double om = k->omega0 + dt * om_dot - GPS_OMEGAE_DOT * e->toe.tow;

  /* Compute the satellite's position in Earth-Centered Earth-Fixed
   * coordiates. */
  pos[0] = x * cos(om) - y * cos(inc) * sin(om);
  pos[1] = x * sin(om) + y * cos(inc) * cos(om);
  pos[2] = y * sin(inc);

  /* Compute the satellite's velocity in Earth-Centered Earth-Fixed
   * coordiates. */
  temp = y_dot * cos(inc) - y * sin(inc) * inc_dot;
  vel[0] = -om_dot * pos[1] + x_dot * cos(om) - temp * sin(om);
  vel[1] = om_dot * pos[0] + x_dot * sin(om) + temp * cos(om);
  vel[2] = y * cos(inc) * inc_dot + y_dot * sin(inc);

  // Note that there is a typo in the reference used for this equation.
  // The reference uses  omega' * x which should be  omeaga' * x'.
  double acc_common_1 = vel[2] * inc_dot - om_dot * x_dot + y * inc_acc * sin(inc) - y_acc * cos(inc) + inc_dot * y_dot * sin(inc);
  double acc_common_2 = x_acc + y * om_dot * inc_dot * sin(inc) - om_dot * y_dot * cos(inc);
  acc[0] = -om_dot * vel[1] + sin(om) * acc_common_1 + cos(om) * acc_common_2;
  acc[1] = om_dot * vel[0] - cos(om) * acc_common_1 + sin(om) * acc_common_2;
  // Note that there is a typo in the reference used for this equation.
  // The reference uses -y * inc^2 which should be -y * inc'^2.
  acc[2] = sin(inc) * (-y * inc_dot * inc_dot + y_acc) + cos(inc) * (y * inc_acc + 2 * inc_dot * y_dot);

  *iode = k->iode;

  return 0;
}

/** Calculate satellite position, velocity and clock offset from ephemeris.
 *
 * Dispatch to internal function for Kepler/XYZ ephemeris depending on
 * constellation.
 *
 * \param e Pointer to an ephemeris structure for the satellite of interest
 * \param t GPS time at which to calculate the satellite state
 * \param pos Array into which to write calculated satellite position [m]
 * \param vel Array into which to write calculated satellite velocity [m/s]
 * \param clock_err Pointer to where to store the calculated satellite clock
 *                  error [s]
 * \param clock_rate_err Pointer to where to store the calculated satellite
 *                       clock error [s/s]
 * \param iodc Issue of data clock [unitless]
 * \param iode Issue of data ephemeris [unitless]
 *
 * \return  0 on success,
 *         -1 if ephemeris is invalid
 */
s8 calc_sat_state(const ephemeris_t *e, const gps_time_t *t,
                  double pos[3], double vel[3], double acc[3],
                  double *clock_err, double *clock_rate_err, u16 *iodc, u8 *iode)
{
  assert(pos != NULL);
  assert(vel != NULL);
  assert(clock_err != NULL);
  assert(clock_rate_err != NULL);
  assert(e != NULL);

  if (!ephemeris_valid(e, t)) {
    log_error_sid(e->sid,
                  "Using invalid or too old ephemeris in calc_sat_state"
                  " (v:%d, fi:%d, [%d, %f]), [%d, %f]",
                  (int)e->valid, (int)e->fit_interval,
                  (int)e->toe.wn, e->toe.tow,
                  (int)t->wn, t->tow);
    return -1;
  }

  return calc_sat_state_n(e, t, pos, vel, acc, clock_err, clock_rate_err, iodc, iode);
}

/** Calculate satellite position, velocity and clock offset from ephemeris
 * without ephemeris validity check.
 *
 * Dispatch to internal function for Kepler/XYZ ephemeris depending on
 * constellation.
 *
 * \param e Pointer to an ephemeris structure for the satellite of interest
 * \param t GPS time at which to calculate the satellite state
 * \param pos Array into which to write calculated satellite position [m]
 * \param vel Array into which to write calculated satellite velocity [m/s]
 * \param clock_err Pointer to where to store the calculated satellite clock
 *                  error [s]
 * \param clock_rate_err Pointer to where to store the calculated satellite
 *                       clock error [s/s]
 * \param iodc Issue of data clock [unitless]
 * \param iode Issue of data ephemeris [unitless]
 *
 * \return  0 on success,
 *         -1 if ephemeris is invalid
 */
s8 calc_sat_state_n(const ephemeris_t *e, const gps_time_t *t,
                    double pos[3], double vel[3], double acc[3],
                    double *clock_err, double *clock_rate_err,
                    u16 *iodc, u8 *iode)
{
  assert(pos != NULL);
  assert(vel != NULL);
  assert(clock_err != NULL);
  assert(clock_rate_err != NULL);
  assert(e != NULL);

  switch (sid_to_constellation(e->sid)) {
  case CONSTELLATION_GPS:
  case CONSTELLATION_BDS2:
    return calc_sat_state_kepler(e, t, pos, vel, acc, clock_err, clock_rate_err, iodc, iode);
  case CONSTELLATION_SBAS:
    return calc_sat_state_xyz(e, t, pos, vel, acc, clock_err, clock_rate_err, iodc, iode);
  case CONSTELLATION_GLO:
    return calc_sat_state_glo(e, t, pos, vel, acc, clock_err, clock_rate_err, iodc, iode);
  case CONSTELLATION_INVALID:
  case CONSTELLATION_COUNT:
  case CONSTELLATION_GAL:
  case CONSTELLATION_QZS:
  default:
    assert(!"Unsupported constellation");
    return -1;
  }
}

/** Calculate the azimuth and elevation of a satellite from a reference
 * position given the satellite ephemeris.
 *
 * \param e  Pointer to an ephemeris structure for the satellite of interest.
 * \param t    GPS time at which to calculate the az/el.
 * \param ref  ECEF coordinates of the reference point from which the azimuth
 *             and elevation is to be determined, passed as [X, Y, Z], all in
 *             meters.
 * \param az   Pointer to where to store the calculated azimuth output [rad].
 * \param el   Pointer to where to store the calculated elevation output [rad].
 * \param check_e set this parameter as "true" if ephemeris validity check
 *                needed, otherwise set as "false"
 * \return  0 on success,
 *         -1 if almanac is not valid or too old
 */
s8 calc_sat_az_el(const ephemeris_t *e, const gps_time_t *t,
                  const double ref[3], double *az, double *el,
                  bool check_e)
{
  double sat_pos[3];
  double sat_vel[3];
  double sat_acc[3];
  u16 iodc;
  u8 iode;
  double clock_err, clock_rate_err;
  s8 ret;
  if (check_e)
    ret = calc_sat_state(e, t, sat_pos, sat_vel, sat_acc, &clock_err, &clock_rate_err, &iodc, &iode);
  else
    ret = calc_sat_state_n(e, t, sat_pos, sat_vel, sat_acc, &clock_err, &clock_rate_err, &iodc, &iode);
  if (ret != 0) {
    return ret;
  }
  wgsecef2azel(sat_pos, ref, az, el);

  return 0;
}

/** Calculate the Doppler shift of a satellite as observed at a reference
 * position given the satellite ephemeris.
 *
 * \param e  Pointer to an ephemeris structure for the satellite of interest.
 * \param t  GPS time at which to calculate the doppler value.
 * \param ref_pos  ECEF coordinates of the reference point from which the
 *                 Doppler is to be determined, passed as [X, Y, Z], all in
 *                 meters.
 * \param ref_vel ECEF speed vector of the receiver, m/s.
 * \param doppler The Doppler shift [Hz].
 * \return  0 on success,
 *         -1 if ephemeris is not valid or too old
 */
s8 calc_sat_doppler(const ephemeris_t* e, const gps_time_t *t,
                    const double ref_pos[3], const double ref_vel[3],
                    double *doppler)
{
  double sat_pos[3];
  double sat_vel[3];
  double sat_acc[3];
  double clock_err, clock_rate_err;
  double vec_ref_sat_pos[3];
  double vec_ref_sat_vel[3];
  u16 iodc;
  u8 iode;

  s8 ret = calc_sat_state(e, t, sat_pos, sat_vel, sat_acc, &clock_err, &clock_rate_err, &iodc, &iode);
  if (ret != 0) {
    return ret;
  }

  /* Find the vector from the reference position to the satellite. */
  vector_subtract(3, sat_pos, ref_pos, vec_ref_sat_pos);

  /* Find the velocity diff between receiver and satellite. */
  vector_add(3, sat_vel, ref_vel, vec_ref_sat_vel);

  /* Find the satellite - receiver velocity projected on the line of sight
   * vector from the reference position to the satellite. */
  double radial_vel = vector_dot(3, vec_ref_sat_pos, vec_ref_sat_vel) /
    vector_norm(3, vec_ref_sat_pos);

  /* Return the Doppler shift. */
  *doppler = sid_to_carr_freq(e->sid) * radial_vel / GPS_C;

  return 0;
}

/** Is this ephemeris usable?
 *
 * \param e Ephemeris struct
 * \param t The current GPS time. This is used to determine the ephemeris age.
 * \return 1 if the ephemeris is valid and not too old.
 *         0 otherwise.
 */
u8 ephemeris_valid(const ephemeris_t *e, const gps_time_t *t)
{
  if (e == NULL || t == NULL) {
    return 0;
  }

  if (IS_GPS(e->sid) || IS_BDS2(e->sid)) {
    return ephemeris_params_valid(e->valid, e->fit_interval, &e->toe,
                                  &e->kepler.toc, t);
  } else {
    return ephemeris_params_valid(e->valid, e->fit_interval, &e->toe, NULL, t);
  }
}

/** Lean version of ephemeris_valid
 * The function allows to avoid passing whole ephemeris
 *
 * \param valid ephemeris Valid flag after decoding
 * \param fit_interval Curve fit interval in seconds
 * \param toe Time from ephemeris reference epoch
 * \param toc Time from ephemeris clock reference epoch. Can be NULL.
 * \param t The current GPS time. This is used to determine the ephemeris age
 * \return 1 if the ephemeris is valid and not too old.
 *         0 otherwise.
 */
u8 ephemeris_params_valid(const u8 valid, const u32 fit_interval,
                          const gps_time_t* toe, const gps_time_t* toc,
                          const gps_time_t *t)
{
  assert(t != NULL);
  assert(toe != NULL);

  /* Seconds from the time from ephemeris reference epoch (toe) */
  double dt_s = gpsdifftime(t, toe);

  if (!valid) {
    return 0;
  }

  if (fit_interval <= 0) {
    log_warn("ephemeris_valid used with 0 e->fit_interval");
    return 0;
  }

  /*
   * Ephemeris did not get time-stammped when it was received.
   */
  if (toe->wn == 0) {
    return 0;
  }

  /* TODO: this doesn't exclude ephemerides older than a week so could be made
   * better. */
  /* If dt is greater than fit_interval / 2 seconds our ephemeris isn't valid. */
  if (fabs(dt_s) > ((u32)fit_interval / 2)) {
    return 0;
  }

  /* Also check GPS ToC */
  if (toc != NULL) {
    if (toc->wn == 0) {
      return 0;
    }

    dt_s = gpsdifftime(t, toc);

    if (fabs(dt_s) > ((u32)fit_interval / 2)) {
      return 0;
    }
  }

  return 1;
}

#define URA_VALUE_TABLE_LEN 16

static const float gps_ura_values[URA_VALUE_TABLE_LEN] = {
  [0]  = 2.0f,
  [1]  = 2.8f,
  [2]  = 4.0f,
  [3]  = 5.7f,
  [4]  = 8.0f,
  [5]  = 11.3f,
  [6]  = 16.0f,
  [7]  = 32.0f,
  [8]  = 64.0f,
  [9]  = 128.0f,
  [10] = 256.0f,
  [11] = 512.0f,
  [12] = 1024.0f,
  [13] = 2048.0f,
  [14] = 4096.0f,
  [15] = 6144.0f,
};

/** Convert a GPS URA index into a value.
*
* \param index URA index.
* \return the URA in meters.
*/
float decode_ura_index(const u8 index)
{
  /* Invalid index */
  if (URA_VALUE_TABLE_LEN < index)
    return INVALID_GPS_URA_VALUE;

  return gps_ura_values[index];
}

/** Convert GPS URA into URA index.
*
* \param ura URA in meters.
* \return URA index.
*/
u8 encode_ura(float ura)
{
  /* Negative URA */
  if (0 > ura)
    return INVALID_GPS_URA_INDEX;

  for (u8 i = 0; i < URA_VALUE_TABLE_LEN; i++) {
    if (gps_ura_values[i] >= ura)
      return i;
  }

  /* No valid URA index found */
  return INVALID_GPS_URA_INDEX;
}

/** Calculate the GPS ephemeris curve fit interval.
*
* \param fit_interval_flag The curve fit interval flag. 0 is 4 hours, 1 is >4 hours.
* \param iodc The IODC value.
* \return the curve fit interval in seconds.
*/
u32 decode_fit_interval(u8 fit_interval_flag, u16 iodc) {
  u8 fit_interval = 4; /* This is in hours */

  if (fit_interval_flag) {
    fit_interval = 6;

    if ((iodc >= 240) && (iodc <= 247)) {
      fit_interval = 8;
    } else if (((iodc >= 248) && (iodc <= 255)) || (iodc == 496)) {
      fit_interval = 14;
    } else if (((iodc >= 497) && (iodc <= 503)) || ((iodc >= 1021) && (iodc <= 1023))) {
      fit_interval = 26;
    } else if ((iodc >= 504) && (iodc <= 510)) {
      fit_interval = 50;
    } else if ((iodc == 511) || ((iodc >= 752) && (iodc <= 756))) {
      fit_interval = 74;
    } else if (iodc == 757) {
      fit_interval = 98;
    }
  }

  return fit_interval * 60 * 60;
}

/** Decode ephemeris from L1 C/A GPS navigation message frames.
 *
 * \note This function does not check for parity errors. You should check the
 *       subframes for parity errors before calling this function.
 *
 * References:
 *   -# IS-GPS-200D, Section 20.3.2 and Figure 20-1
 *
 * \param frame_words Array containing words 3 through 10 of subframes
 *                    1, 2 and 3. Word is in the 30 LSBs of the u32.
 * \param e Pointer to an ephemeris struct to fill in.
 * \param tot_tow TOW for time of transmission
 */
void decode_ephemeris(const u32 frame_words[3][8], ephemeris_t *e, double tot_tow)
{
  assert(frame_words != NULL);
  assert(e != NULL);
  assert(IS_GPS(e->sid));
  ephemeris_kepler_t *k = &e->kepler;

  /* Subframe 1: WN, URA, SV health, T_GD, IODC, t_oc, a_f2, a_f1, a_f0 */

  /* GPS week number (mod 1024): Word 3, bits 1-10 */
  u16 wn_raw = frame_words[0][3-3] >> (30-10) & 0x3FF;

  /*
   * The ten MSBs of word three shall contain the ten LSBs of the
   * Week Number as defined in 3.3.4. These ten bits shall be a modulo
   * 1024 binary representation of the current GPS week number
   * at the start of the data set transmission interval. <<<<< IMPORTANT !!!
   */
  e->toe.wn = gps_adjust_week_cycle(wn_raw, GPS_WEEK_REFERENCE);

  /* t_oe: Word 10, bits 1-16 */
  e->toe.tow = (frame_words[1][10-3] >> (30-16) & 0xFFFF) * GPS_LNAV_EPH_SF_TOE;

  bool toe_valid = gps_time_valid(&e->toe);
  if (toe_valid) {
    /* Match TOE week number with the time of transmission, fixes the case
     * near week roll-over where next week's ephemeris still has current week's
     * week number. */
    gps_time_t tot = {.wn = e->toe.wn, .tow = tot_tow};
    gps_time_match_weeks(&e->toe, &tot);
  } else {
    /* Invalid TOE, most likely TOW overflow.
     * Continue to decode the ephemeris, but mark it invalid at the end */
    log_warn_sid(e->sid,
               "Latest ephemeris has faulty TOE: wn %d, tow %f."
               " Invalidating ephemeris.", e->toe.wn, e->toe.tow);
  }

  k->toc.wn = e->toe.wn;

  /* URA: Word 3, bits 13-16 */
  /* Value of 15 is unhealthy */
  u8 ura_index = frame_words[0][3-3] >> (30-16) & 0xF;
  e->ura = decode_ura_index(ura_index);
  log_debug_sid(e->sid, "URA = index %d, value %.1f", ura_index, e->ura);

  /* NAV data and signal health bits: Word 3, bits 17-22 */
  e->health_bits = frame_words[0][3-3] >> (30-22) & 0x3F;
  log_debug_sid(e->sid, "Health bits = 0x%02" PRIx8, e->health_bits);

  /* t_gd: Word 7, bits 17-24 */
  k->tgd = (s8)(frame_words[0][7-3] >> (30-24) & 0xFF) * GPS_LNAV_EPH_SF_TGD;

  /* iodc: Word 3, bits 23-24 and word 8, bits 1-8 */
  k->iodc = ((frame_words[0][3-3] >> (30-24) & 0x3) << 8)
          | (frame_words[0][8-3] >> (30-8) & 0xFF);

  /* t_oc: Word 8, bits 8-24 */
  k->toc.tow = (frame_words[0][8-3] >> (30-24) & 0xFFFF) * GPS_LNAV_EPH_SF_TOC;

  /* a_f2: Word 9, bits 1-8 */
  k->af2 = (s8)(frame_words[0][9-3] >> (30-8) & 0xFF) * GPS_LNAV_EPH_SF_AF2;

  /* a_f1: Word 9, bits 9-24 */
  k->af1 = (s16)(frame_words[0][9-3] >> (30-24) & 0xFFFF) * GPS_LNAV_EPH_SF_AF1;

  /* a_f0: Word 10, bits 1-22 */
  k->af0 = sign_extend22(frame_words[0][10-3] >> (30-22) & 0x3FFFFF) * GPS_LNAV_EPH_SF_AF0;

  /* Subframe 2: IODE, crs, dn, m0, cuc, ecc, cus, sqrta, toe, fit_interval */

  /* iode: Word 3, bits 1-8 */
  u8 iode_sf2 = frame_words[1][3-3] >> (30-8) & 0xFF;

  /* crs: Word 3, bits 9-24 */
  k->crs = (s16)(frame_words[1][3-3] >> (30-24) & 0xFFFF) * GPS_LNAV_EPH_SF_CRS;

  /* dn: Word 4, bits 1-16 */
  k->dn = (s16)(frame_words[1][4-3] >> (30-16) & 0xFFFF) * (GPS_LNAV_EPH_SF_DN * GPS_PI);

  /* m0: Word 4, bits 17-24 and word 5, bits 1-24 */
  k->m0 = (s32)(((frame_words[1][4-3] >> (30-24) & 0xFF) << 24) |
                 (frame_words[1][5-3] >> (30-24) & 0xFFFFFF)) *
          (GPS_LNAV_EPH_SF_M0 * GPS_PI);

  /* cuc: Word 6, bits 1-16 */
  k->cuc = (s16)(frame_words[1][6-3] >> (30-16) & 0xFFFF) * GPS_LNAV_EPH_SF_CUC;

  /* ecc: Word 6, bits 17-24 and word 7, bits 1-24 */
  k->ecc = (u32)(((frame_words[1][6-3] >> (30-24) & 0xFF) << 24) |
                  (frame_words[1][7-3] >> (30-24) & 0xFFFFFF)) *
            GPS_LNAV_EPH_SF_ECC;

  /* cus: Word 8, bits 1-16 */
  k->cus = (s16)(frame_words[1][8-3] >> (30-16) & 0xFFFF) *
           GPS_LNAV_EPH_SF_CUS;

  /* sqrta: Word 8, bits 17-24 and word 9, bits 1-24 */
  k->sqrta = (u32)(((frame_words[1][8-3] >> (30-24) & 0xFF) << 24) |
                    (frame_words[1][9-3] >> (30-24) & 0xFFFFFF)) *
             GPS_LNAV_EPH_SF_SQRTA;

  /* fit_interval_flag: Word 10, bit 17 */
  u8 fit_interval_flag = frame_words[1][10-3] >> (30-17) & 0x1;
  e->fit_interval = decode_fit_interval(fit_interval_flag, k->iodc);
  log_debug_sid(e->sid, "Fit interval = %" PRIu32, e->fit_interval);

  /* Subframe 3: cic, omega0, cis, inc, crc, w, omegadot, IODE, inc_dot */

  /* cic: Word 3, bits 1-16 */
  k->cic = (s16)(frame_words[2][3-3] >> (30-16) & 0xFFFF) *
           GPS_LNAV_EPH_SF_CIC;

  /* omega0: Word 3, bits 17-24 and word 4, bits 1-24 */
  k->omega0 =(s32)(((frame_words[2][3-3] >> (30-24) & 0xFF) << 24) |
                    (frame_words[2][4-3] >> (30-24) & 0xFFFFFF)) *
             (GPS_LNAV_EPH_SF_OMEGA0 * GPS_PI);

  /* cis: Word 5, bits 1-16 */
  k->cis = (s16)(frame_words[2][5-3] >> (30-16) & 0xFFFF) * GPS_LNAV_EPH_SF_CIS;

  /* inc (i0): Word 5, bits 17-24 and word 6, bits 1-24 */
  k->inc = (s32)(((frame_words[2][5-3] >> (30-24) & 0xFF) << 24) |
                  (frame_words[2][6-3] >> (30-24) & 0xFFFFFF)) *
           (GPS_LNAV_EPH_SF_I0 * GPS_PI);

  /* crc: Word 7, bits 1-16 */
  k->crc = (s16)(frame_words[2][7-3] >> (30-16) & 0xFFFF) *
           GPS_LNAV_EPH_SF_CRC;

  /* w (omega): Word 7, bits 17-24 and word 8, bits 1-24 */
  k->w = (s32)(((frame_words[2][7-3] >> (30-24) & 0xFF) << 24) |
                (frame_words[2][8-3] >> (30-24) & 0xFFFFFF)) *
         (GPS_LNAV_EPH_SF_W * GPS_PI);

  /* Omega_dot: Word 9, bits 1-24 */
  k->omegadot = sign_extend24(frame_words[2][9-3] >> (30-24) & 0xFFFFFF) *
                (GPS_LNAV_EPH_SF_OMEGADOT * GPS_PI);

  /* iode: Word 10, bits 1-8 */
  k->iode = frame_words[2][10-3] >> (30-8) & 0xFF;

  /* inc_dot (IDOT): Word 10, bits 9-22 */
  k->inc_dot = sign_extend14(frame_words[2][10-3] >> (30-22) & 0x3FFF) *
               (GPS_LNAV_EPH_SF_IDOT * GPS_PI);

  /* Both IODEs and IODC (8 LSBs) must match */
  log_debug_sid(e->sid,
                "Check ephemeris. IODC = 0x%03" PRIX16 " IODE = 0x%02" PRIX8
                " and 0x%02" PRIX8 ".", k->iodc, iode_sf2, k->iode);

  bool iode_valid = (iode_sf2 == k->iode) && (k->iode == (k->iodc & 0xFF));
  if (!iode_valid) {
    log_warn_sid(e->sid,
                 "Latest ephemeris has IODC/IODE mismatch."
                 " Invalidating ephemeris.");
  }

  e->valid = iode_valid && toe_valid;
}

static bool ephemeris_xyz_equal(const ephemeris_xyz_t *a,
                                const ephemeris_xyz_t *b)
{
  return (a->a_gf0 == b->a_gf0) &&
         (a->a_gf1 == b->a_gf1) &&
         (memcmp(a->pos, b->pos, sizeof(a->pos)) == 0) &&
         (memcmp(a->vel, b->vel, sizeof(a->vel)) == 0) &&
         (memcmp(a->acc, b->acc, sizeof(a->acc)) == 0);
}

static bool ephemeris_kepler_equal(const ephemeris_kepler_t *a,
                                   const ephemeris_kepler_t *b)
{
  return (a->iodc == b->iodc) &&
         (a->iode == b->iode) &&
         (a->tgd == b->tgd) &&
         (a->crs == b->crs) &&
         (a->crc == b->crc) &&
         (a->cuc == b->cuc) &&
         (a->cus == b->cus) &&
         (a->cic == b->cic) &&
         (a->cis == b->cis) &&
         (a->dn == b->dn) &&
         (a->m0 == b->m0) &&
         (a->ecc == b->ecc) &&
         (a->sqrta == b->sqrta) &&
         (a->omega0 == b->omega0) &&
         (a->omegadot == b->omegadot) &&
         (a->w == b->w) &&
         (a->inc == b->inc) &&
         (a->inc_dot == b->inc_dot) &&
         (a->af0 == b->af0) &&
         (a->af1 == b->af1) &&
         (a->af2 == b->af2) &&
         (a->toc.wn == b->toc.wn) &&
         (a->toc.tow == b->toc.tow);
}

static bool ephemeris_glo_equal(const ephemeris_glo_t *a,
                                const ephemeris_glo_t *b)
{
  return (a->gamma == b->gamma) &&
         (a->tau == b->tau) &&
         (a->d_tau == b->d_tau) &&
         (a->iod == b->iod) &&
         (a->fcn == b->fcn) &&
         (memcmp(a->pos, b->pos, sizeof(a->pos)) == 0) &&
         (memcmp(a->vel, b->vel, sizeof(a->vel)) == 0) &&
         (memcmp(a->acc, b->acc, sizeof(a->acc)) == 0);
}

/** Are the two ephemerides the same?
 *
 * \param a First ephemeris
 * \param b Second ephemeris
 * \return true if they are equal
 */
bool ephemeris_equal(const ephemeris_t *a, const ephemeris_t *b)
{
  if (!sid_is_equal(a->sid, b->sid) ||
      (a->ura != b->ura) ||
      (a->fit_interval != b->fit_interval) ||
      (a->valid != b->valid) ||
      (a->health_bits != b->health_bits) ||
      (a->toe.wn != b->toe.wn) ||
      (a->toe.tow != b->toe.tow))
    return false;

  switch (sid_to_constellation(a->sid)) {
  case CONSTELLATION_GPS:
  case CONSTELLATION_BDS2:
    return ephemeris_kepler_equal(&a->kepler, &b->kepler);
  case CONSTELLATION_SBAS:
    return ephemeris_xyz_equal(&a->xyz, &b->xyz);
  case CONSTELLATION_GLO:
    return ephemeris_glo_equal(&a->glo, &b->glo);
  case CONSTELLATION_INVALID:
  case CONSTELLATION_COUNT:
  case CONSTELLATION_GAL:
  case CONSTELLATION_QZS:
  default:
    assert(!"Unsupported constellation");
    return false;
  }
}

/** Check if this this ephemeris is healthy
 *
 * \param ephe pointer to ephemeris to check
 * \param code signal code, ephe->sid can't be used as for example L2CM uses
 *             L1CA ephes
 * \return true if the ephemeris is healthy
 *         false otherwise
 */
bool ephemeris_healthy(const ephemeris_t* ephe, const code_t code) {
  /* Presume healthy */
  bool ret = true;

  if (!ephe->valid) {
    /* If we don't yet have an ephemeris, assume satellite is healthy */
    /* Otherwise we will stop tracking the sat and never find out */
    return ret;
  }

  switch (code_to_constellation(code)) {
    case CONSTELLATION_GPS:
      if (encode_ura(ephe->ura) > MAX_ALLOWED_GPS_URA_IDX) {
        ret = false;
        /* Satellite is not healthy, no reason to check further */
        break;
      }

      ret = check_6bit_health_word(ephe->health_bits, code);

      break;

    case CONSTELLATION_SBAS:
    case CONSTELLATION_GLO:
    case CONSTELLATION_BDS2:
      ret = (0 == ephe->health_bits);
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      assert(!"Unsupported constellation");
      ret = false;
      break;
  }

  return ret;
}

/** Get the ephemeris iod. For GPS, returns IODE
 *
 * \param a eph Ephemeris
 * \return Issue of Data
 */
u8 get_ephemeris_iod(const ephemeris_t *eph)
{
  switch (sid_to_constellation(eph->sid)) {
    case CONSTELLATION_GPS:
    case CONSTELLATION_BDS2:
      return eph->kepler.iode;
    case CONSTELLATION_GLO:
      return eph->glo.iod;
    case CONSTELLATION_INVALID:
    case CONSTELLATION_SBAS:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      assert(!"Unsupported constellation");
          return 0;
  }
}

/** Get the time group delay to be applied to the satellite clock correction
 * \param eph Ephemeris
 * \param sid Sid of the signal to correct
 * \return Applied group delay correction
 */
double get_tgd_correction(const ephemeris_t *eph, const gnss_signal_t *sid){
  double frequency, gamma;
  assert(sid_to_constellation(eph->sid) == sid_to_constellation(*sid));
  switch (sid_to_constellation(*sid)) {
    case CONSTELLATION_GPS:
      /* sat_clock_error = iono_free_clock_error - (f_1 / f)^2 * TGD. */
      frequency = sid_to_carr_freq(*sid);
      gamma = GPS_L1_HZ * GPS_L1_HZ / (frequency * frequency);
      return eph->kepler.tgd * gamma;
    case CONSTELLATION_BDS2:
      /* sat_clock_error = iono_free_clock_error - (f_1 / f)^2 * TGD. */
      frequency = sid_to_carr_freq(*sid);
      gamma = BDS2_B11_HZ * BDS2_B11_HZ / (frequency * frequency);
      return eph->kepler.tgd * gamma;
    case CONSTELLATION_GLO:
      /* As per GLO ICD v5.1 2008:
         d_tau = t_f2 - t_f1 -> t_f1 = t_f2 - d_tau.
         As clock_err is added to pseudorange,
         d_tau has to be applied with negative sign. */
      if (CODE_GLO_L2OF == sid->code) {
        return eph->glo.d_tau;
      }
      if (CODE_GLO_L1OF == sid->code) {
        return 0.0;
      }
    case CONSTELLATION_INVALID:
    case CONSTELLATION_SBAS:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      assert(!"Unsupported constellation");
          return 0;
  }
}

/** \} */
