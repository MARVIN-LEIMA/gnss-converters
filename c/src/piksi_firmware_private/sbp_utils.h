/*
 * Copyright (C) 2014, 2016 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SBP_UTILS_H
#define SWIFTNAV_SBP_UTILS_H

#include <libsbp/common.h>
#include <libsbp/gnss.h>
#include <libsbp/navigation.h>
#include <libsbp/ndb.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/system.h>
//include <libswiftnav/almanac.h>
//include <libswiftnav/ephemeris.h>
//include <libswiftnav/gnss_time.h>
//include <libswiftnav/sbas_raw_data.h>
//include <libswiftnav/signal.h>
//include <libswiftnav/single_epoch_solver.h>

//include "nav_msg/cnav_msg.h"
//include "obs_bias/obs_bias.h"

typedef struct {
  union {
    msg_ephemeris_gps_t gps;
    msg_ephemeris_sbas_t sbas;
    msg_ephemeris_glo_t glo;
  };
} msg_ephemeris_t;

typedef struct {
  u16 msg_id;
  u16 size;
} msg_info_t;

typedef union {
  msg_almanac_gps_t gps;
  msg_almanac_glo_t glo;
} msg_almanac_t;

typedef enum {
  NDB_EVENT_UNKNOWN = 0,
  NDB_EVENT_STORE = 1,
  NDB_EVENT_FETCH = 2,
  NDB_EVENT_ERASE = 3,
} ndb_event_t;

typedef enum {
  NDB_EVENT_OTYPE_UNKNOWN = 0,
  NDB_EVENT_OTYPE_EPHEMERIS = 1,
  NDB_EVENT_OTYPE_ALMANAC = 2,
  NDB_EVENT_OTYPE_ALMANAC_WN = 3,
  NDB_EVENT_OTYPE_IONO = 4,
  NDB_EVENT_OTYPE_L2C_CAP = 5,
  NDB_EVENT_OTYPE_LGF = 6,
  NDB_EVENT_OTYPE_UTC_PARAMS = 7,
} ndb_event_obj_type_t;

#define NDB_EVENT_SENDER_ID_VOID 0

#if 0
void sbp_init_gps_time(msg_gps_time_t *gps_time, gps_time_t *t);
void sbp_init_utc_time(msg_utc_time_t *utc_time, gps_time_t *t);
void sbp_init_pos_llh(msg_pos_llh_t *pos_llh, gps_time_t *t);
void sbp_init_pos_ecef(msg_pos_ecef_t *pos_ecef, gps_time_t *t);
void sbp_init_vel_ned(msg_vel_ned_t *vel_ned, gps_time_t *t);
void sbp_init_vel_ecef(msg_vel_ecef_t *vel_ecef, gps_time_t *t);
void sbp_init_sbp_dops(msg_dops_t *sbp_dops, gps_time_t *t);
void sbp_init_age_corrections(msg_age_corrections_t *age_corrections,
                              gps_time_t *t);
void sbp_init_dgnss_status(msg_dgnss_status_t *dgnss_status);
void sbp_init_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, gps_time_t *t);
void sbp_init_baseline_ned(msg_baseline_ned_t *baseline_ned, gps_time_t *t);
void sbp_init_baseline_heading(msg_baseline_heading_t *baseline_heading,
                               gps_time_t *t);
void sbp_init_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov, gps_time_t *t);
void sbp_init_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov, gps_time_t *t);
void sbp_init_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov, gps_time_t *t);
void sbp_init_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov, gps_time_t *t);

void sbp_make_gps_time(msg_gps_time_t *t_out, const gps_time_t *t_in, u8 flags);
void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh,
                           const double llh[3],
                           double h_accuracy,
                           double v_accuracy,
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags);
void sbp_make_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov,
                          const double llh[3],
                          const double llh_cov[6],
                          const gps_time_t *gps_t,
                          u8 n_sats_used,
                          u8 flags);
void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef,
                            const double ecef[3],
                            double accuracy,
                            const gps_time_t *gps_t,
                            u8 n_sats_used,
                            u8 flags);
void sbp_make_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov,
                           const double ecef[3],
                           const double ecef_cov[6],
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags);
void sbp_make_vel_ned(msg_vel_ned_t *vel_ned,
                      const double v_ned[3],
                      double h_accuracy,
                      double v_accuracy,
                      const gps_time_t *gps_t,
                      u8 n_sats_used,
                      u8 flags);
void sbp_make_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov,
                          const double v_ned[3],
                          const double ned_cov[6],
                          const gps_time_t *gps_t,
                          u8 n_sats_used,
                          u8 flags);
void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef,
                       const double v_ecef[3],
                       double accuracy,
                       const gps_time_t *gps_t,
                       u8 n_sats_used,
                       u8 flags);
void sbp_make_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov,
                           const double v_ecef[3],
                           const double ecef_cov[6],
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags);
void sbp_make_dops(msg_dops_t *dops_out,
                   const dops_t *dops_in,
                   u32 tow,
                   u8 flags);
void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef,
                            const gps_time_t *t,
                            u8 n_sats,
                            const double b_ecef[3],
                            double accuracy,
                            u8 flags);
void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned,
                           const gps_time_t *t,
                           u8 n_sats,
                           const double b_ned[3],
                           double h_accuracy,
                           double v_accuracy,
                           u8 flags);
void sbp_make_heading(msg_baseline_heading_t *baseline_heading,
                      const gps_time_t *t,
                      const double heading,
                      u8 n_sats_used,
                      u8 flags);
void sbp_make_age_corrections(msg_age_corrections_t *age_corrections,
                              const gps_time_t *t,
                              double propagation_time);
void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status,
                           u8 num_sats,
                           double obs_latency,
                           u8 flags);
void sbp_make_utc_time(msg_utc_time_t *t_out, const gps_time_t *t_in, u8 flags);
void sbp_send_ndb_event(u8 event,
                        u8 obj_type,
                        u8 result,
                        u8 data_source,
                        const gnss_signal_t *object_sid,
                        const gnss_signal_t *src_sid,
                        u16 sender);
#endif

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK

#define MSG_OBS_P_MULTIPLIER ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER ((float)4)
#define MSG_OBS_LF_OVERFLOW (1 << 8)
#define MSG_OBS_DF_OVERFLOW (1 << 8)
#define MSG_OBS_LF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_DF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID ((u8)(1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID ((u8)(1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN ((u8)(1 << 2))
#define MSG_OBS_FLAGS_MEAS_DOPPLER_VALID ((u8)(1 << 3))
#define MSG_OBS_FLAGS_RAIM_EXCLUSION ((u8)(1 << 7))

#define MSG_HEADING_SCALE_FACTOR 1000.0

#if 0
#define MSG_FORWARD_SENDER_ID 0

void unpack_obs_header(const observation_header_t *msg,
                       gps_time_t *t,
                       u8 *total,
                       u8 *count);

void pack_obs_header(const gps_time_t *t,
                     u8 total,
                     u8 count,
                     observation_header_t *msg);

u8 nm_flags_to_sbp(nav_meas_flags_t from);
nav_meas_flags_t nm_flags_from_sbp(u8 from);

void unpack_obs_content(const packed_obs_content_t *msg,
                        double *P,
                        double *L,
                        double *D,
                        double *cn0,
                        double *lock_time,
                        nav_meas_flags_t *flags,
                        gnss_signal_t *sid);

s8 pack_obs_content(double P,
                    double L,
                    double D,
                    double cn0,
                    double lock_time,
                    nav_meas_flags_t flags,
                    gnss_signal_t sid,
                    packed_obs_content_t *msg);

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e);

msg_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg);

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8 *, void *));

msg_info_t pack_almanac(const almanac_t *a, msg_almanac_t *msg);

/** Value specifying the size of the SBP framing */
#define SBP_FRAMING_SIZE_BYTES 8
/** Value defining maximum SBP packet size */
#define SBP_FRAMING_MAX_PAYLOAD_SIZE 255

gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from);
sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from);

void sbp_send_iono(const ionosphere_t *iono);
void sbp_send_l2c_capabilities(const u32 *l2c_cap);
void sbp_send_group_delay(const cnav_msg_t *cnav);
void sbp_pack_sbas_raw_data(const gnss_signal_t sid,
                            u32 tow_ms,
                            u8 msg,
                            const u8 *decoded,
                            msg_sbas_raw_t *sbas_raw_msg);
void unpack_sbas_raw_data(const msg_sbas_raw_t *m, sbas_raw_data_t *d);
#endif

#endif /* SWIFTNAV_SBP_UTILS_H */
