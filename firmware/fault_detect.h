/*
 * fault_detect.h — plausibility monitoring for the FOC sensor set.
 *
 * Production philosophy (same as a BMS): cheap invariant checks each
 * ISR, debounced so transients don't false-trigger, latched flags the
 * application layer decides how to react to.
 *
 * Checks:
 *   FD_CURRENT_SUM  ia+ib+ic must be ~0 (isolated neutral / KCL) — a
 *                   sensor offset violates this within one sample
 *   FD_CURRENT_MAG  |i_dq| must stay under i_max * margin
 *   FD_ENCODER      angle increment per sample must match w_e * ts
 */

#ifndef FAULT_DETECT_H
#define FAULT_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

#define FD_CURRENT_SUM (1u << 0)
#define FD_CURRENT_MAG (1u << 1)
#define FD_ENCODER     (1u << 2)

typedef struct {
    float ts;            /* ISR period [s]                          */
    float sum_thresh;    /* [A]   |ia+ib+ic| limit                  */
    float mag_thresh;    /* [A]   |i_dq| limit (i_max * margin)     */
    float ang_thresh;    /* [rad] |d_theta - w_e*ts| limit          */
    int   debounce;      /* consecutive violations before latching  */
} fd_params_t;

typedef struct {
    fd_params_t par;
    float theta_prev;
    int   have_prev;
    int   cnt_sum, cnt_mag, cnt_enc;
    unsigned flags;      /* latched fault bits                      */
} fd_t;

void fd_init(fd_t *fd, const fd_params_t *par);

/* One ISR-rate update; returns the latched fault bitmask. */
unsigned fd_step(fd_t *fd, float ia, float ib, float ic,
                 float theta_e, float w_e, float id, float iq);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_DETECT_H */
