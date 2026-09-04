/*
 * speed_ctrl.h — outer speed loop of the FOC cascade.
 *
 * Runs at a divided rate (e.g. 1 kHz vs the 10 kHz current ISR) and
 * commands iq_ref into the current loop, limited to the machine current
 * limit. Symmetric-optimum tuning; first-order reference prefilter
 * cancels the PI-zero overshoot on reference steps while keeping full
 * load-disturbance stiffness.
 */

#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;        /* [A/(rad/s)]                              */
    float ki;        /* [A/(rad s)]  (= kp/Ti)                   */
    float ts;        /* speed-loop sample time [s]               */
    float t_filt;    /* reference prefilter time constant [s];
                        0 disables the prefilter                 */
    float iq_max;    /* symmetric current limit [A]              */
} spd_params_t;

typedef struct {
    spd_params_t par;
    foc_pi_t pi;
    float w_ref_f;   /* prefiltered reference state [rad/s]      */
} spd_t;

void spd_init(spd_t *s, const spd_params_t *par);

/* w_ref, w_m: mechanical speed [rad/s]. Returns iq_ref [A], clamped to
 * +/- iq_max with anti-windup. */
float spd_step(spd_t *s, float w_ref, float w_m);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_CTRL_H */
