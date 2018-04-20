/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_OBSERVATION_H
#define LIBSWIFTNAV_OBSERVATION_H

//#include <libswiftnav/almanac.h>
//#include <libswiftnav/common.h>
//#include <libswiftnav/ephemeris.h>
//#include <libswiftnav/gnss_time.h>
//#include <libswiftnav/nav_meas.h>
//#include <libswiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Define Time types - currently aligned to SBP messages */
#define NO_TIME          0
#define GNSS_TIME        1
#define TIME_SOURCE_MASK 0x07 /* Bits 0-2 */
#define DEFAULT_UTC      0
#define NVM_UTC          1
#define DECODED_UTC      2
#define UTC_SOURCE_MASK  0x18 /* Bits 3-4 */

/* Define Position types - currently aligned to SBP messages */
#define NO_POSITION         0
#define SPP_POSITION        1
#define DGNSS_POSITION      2
#define FLOAT_POSITION      3
#define FIXED_POSITION      4
/* 5 reserved for dead reckoning */
#define SBAS_POSITION       6
#define POSITION_MODE_MASK  0x07 /* Bits 0-2 */
#define RAIM_REPAIR_FLAG    0x80 /* Bit 7 */

/* Define Velocity types - currently aligned to SBP messages */
#define NO_VELOCITY               0
#define MEASURED_DOPPLER_VELOCITY 1
#define COMPUTED_DOPPLER_VELOCITY 2
#define VELOCITY_MODE_MASK        0x07 /* Bits 0-2 */

#if 0
typedef struct {
  double pseudorange;        /**< Single differenced raw pseudorange [m] */
  double carrier_phase;      /**< Single differenced raw carrier phase [cycle]*/
  double measured_doppler;   /**< Single differenced raw measured doppler [Hz] */
  double computed_doppler;   /**< Single differenced raw computed doppler [Hz] */
  double sat_pos[3];         /**< ECEF XYZ of ephemeris satellite position [m]*/
  double sat_vel[3];         /**< ECEF XYZ of ephemeris satellite velocity [m]*/
  double cn0;                /**< The lowest C/N0 of the two single differenced
                              *   observations [dB Hz] */
  double rover_lock_time;    /**< The rover lock time [s] */
  double base_lock_time;     /**< The base lock time [s] */
  gnss_signal_t sid;         /**< SV signal identifier */
  nav_meas_flags_t flags;    /**< Measurement flags: see nav_meas.h */
} sdiff_t;

int cmp_sdiff(const void *a_, const void *b_);
int cmp_amb(const void *a_, const void *b_);
int cmp_amb_sdiff(const void *a_, const void *b_);
int cmp_sdiff_sid(const void *a_, const void *b_);
int cmp_amb_sid(const void *a_, const void *b_);

u8 single_diff(const u8 n_a, const navigation_measurement_t *m_a,
               const u8 n_b, const navigation_measurement_t *m_b,
               sdiff_t *sds);

bool has_mixed_l2_obs(u8 n, navigation_measurement_t *nav_meas);

typedef bool (*navigation_measurement_predicate_f)(navigation_measurement_t);

typedef bool (*navigation_measurement_predicate_extra_f)
  (navigation_measurement_t, void *);

void filter_base_meas(u8 *n, navigation_measurement_t nav_meas[]);

/* Given an array of measurements `nav_meas` containing `n` elements,
 * remove all elements for which the selection function `predicate`
 * returns `false`.  The remaining elements (for which `predicate`
 * returned `true`) will be in their original order.  Returns the
 * number of elements in the array after filtering.
 */
u8 filter_nav_meas(u8 n, navigation_measurement_t nav_meas[],
                   navigation_measurement_predicate_f predicate);

/* Given an array of measurements `nav_meas` containing `n` elements,
 * remove all elements for which the selection function `predicate`
 * returns `false`.  The remaining elements (for which `predicate`
 * returned `true`) will be in their original order. The given pointer
 * `extra_data` will be passed as the second argument to the selection
 * function on each call.  Returns the number of elements in the array
 * after filtering.
 */
u8 filter_nav_meas_extra(u8 n, navigation_measurement_t nav_meas[],
                         navigation_measurement_predicate_extra_f predicate,
                         void *extra_data);

int cmp_sid_sdiff(const void *a, const void *b);

u8 make_propagated_sdiffs(const u8 n_local,
                          const navigation_measurement_t *m_local,
                          const u8 n_remote,
                          const navigation_measurement_t *m_remote,
                          const double remote_pos_ecef[3],
                          sdiff_t *sds);

u8 filter_sdiffs(u8 num_sdiffs, sdiff_t *sdiffs, u8 num_sats_to_drop,
                 gnss_signal_t *sats_to_drop);

void debug_sdiff(sdiff_t sd);
void debug_sdiffs(u8 n, sdiff_t *sds);

s8 calc_sat_clock_corrections(u8 n_channels,
                              navigation_measurement_t *nav_meas[],
                              const ephemeris_t *e[]);
#endif

#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */

#endif /* LIBSWIFTNAV_OBSERVATION_H */
