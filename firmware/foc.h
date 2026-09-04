/*
 * foc.h — portable field-oriented control core for a PMSM.
 *
 * Flashable-shaped: no malloc, no OS, no HAL, no float promotion
 * surprises. All state lives in caller-provided structs; foc_step() is
 * designed to be called from a PWM-rate current ISR.
 *
 * Conventions:
 *  - amplitude-invariant Clarke transform (dq currents = peak phase A)
 *  - theta_e = electrical rotor angle [rad], d-axis aligned with PM flux
 *  - SVPWM linear region: |v_alphabeta| <= v_dc / sqrt(3)
 */

#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- PI controller with conditional anti-windup ---- */

typedef struct {
    float kp;
    float ki;      /* [1/s]; integrated with backward Euler * ts        */
    float ts;      /* sample time [s]                                   */
    float integ;   /* integrator state [output units]                   */
} foc_pi_t;

/* One PI update. out_min/out_max clamp the output; the integrator only
 * advances when the unclamped output is inside the limits or the error
 * drives it back inside (conditional integration anti-windup). */
float foc_pi_step(foc_pi_t *pi, float err, float out_min, float out_max);

/* ---- reference-frame transforms ---- */

void foc_clarke(float ia, float ib, float ic, float *ialpha, float *ibeta);
void foc_park(float ialpha, float ibeta, float sin_th, float cos_th,
              float *id, float *iq);
void foc_inv_park(float vd, float vq, float sin_th, float cos_th,
                  float *valpha, float *vbeta);

/* ---- SVPWM ----
 * v_alphabeta -> per-phase duty cycles in [0,1] via min-max common-mode
 * injection (equivalent to conventional space-vector modulation). */
void foc_svpwm(float valpha, float vbeta, float v_dc,
               float *da, float *db, float *dc);

/* ---- complete current-loop step ---- */

typedef struct {
    float rs;      /* stator resistance [ohm]      */
    float ld;      /* d inductance [H]             */
    float lq;      /* q inductance [H]             */
    float psi_f;   /* PM flux linkage [Wb]         */
    float kp_d, ki_d;
    float kp_q, ki_q;
    float ts;      /* ISR period [s]               */
} foc_params_t;

typedef struct {
    foc_params_t par;
    foc_pi_t pi_d;
    foc_pi_t pi_q;
} foc_t;

typedef struct {
    float ia, ib, ic;   /* measured phase currents [A]        */
    float theta_e;      /* electrical angle [rad]             */
    float w_e;          /* electrical speed [rad/s]           */
    float id_ref;       /* current references [A]             */
    float iq_ref;
    float v_dc;         /* DC-link voltage [V]                */
} foc_in_t;

typedef struct {
    float duty_a, duty_b, duty_c;
    /* debug taps for the co-sim / analysis */
    float id, iq;
    float vd, vq;
    int   sat;          /* 1 if the voltage limit clipped vq  */
} foc_out_t;

void foc_init(foc_t *f, const foc_params_t *par);
void foc_step(foc_t *f, const foc_in_t *in, foc_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
