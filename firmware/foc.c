#include "foc.h"

#include <math.h>

#define ONE_OVER_SQRT3 0.57735026918962576f

/* ---- PI with conditional anti-windup ---- */

float foc_pi_step(foc_pi_t *pi, float err, float out_min, float out_max)
{
    float u = pi->kp * err + pi->integ;

    if (u > out_max) {
        /* saturated high: integrate only if error pulls output down */
        if (err < 0.0f) pi->integ += pi->ki * pi->ts * err;
        return out_max;
    }
    if (u < out_min) {
        if (err > 0.0f) pi->integ += pi->ki * pi->ts * err;
        return out_min;
    }
    pi->integ += pi->ki * pi->ts * err;
    return u;
}

/* ---- transforms ---- */

void foc_clarke(float ia, float ib, float ic, float *ialpha, float *ibeta)
{
    /* amplitude-invariant: ialpha tracks phase-A peak */
    *ialpha = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
    *ibeta  = (ib - ic) * ONE_OVER_SQRT3;
}

void foc_park(float ialpha, float ibeta, float sin_th, float cos_th,
              float *id, float *iq)
{
    *id =  ialpha * cos_th + ibeta * sin_th;
    *iq = -ialpha * sin_th + ibeta * cos_th;
}

void foc_inv_park(float vd, float vq, float sin_th, float cos_th,
                  float *valpha, float *vbeta)
{
    *valpha = vd * cos_th - vq * sin_th;
    *vbeta  = vd * sin_th + vq * cos_th;
}

/* ---- SVPWM via min-max common-mode injection ---- */

void foc_svpwm(float valpha, float vbeta, float v_dc,
               float *da, float *db, float *dc)
{
    /* inverse Clarke to phase voltage references */
    float va = valpha;
    float vb = -0.5f * valpha + (0.5f * 1.7320508075688772f) * vbeta;
    float vc = -0.5f * valpha - (0.5f * 1.7320508075688772f) * vbeta;

    /* centering the envelope between the bus rails = space-vector
     * modulation; buys sqrt(3)/2 -> 1.0 utilization vs sine PWM */
    float vmax = va > vb ? (va > vc ? va : vc) : (vb > vc ? vb : vc);
    float vmin = va < vb ? (va < vc ? va : vc) : (vb < vc ? vb : vc);
    float vcm = 0.5f * (vmax + vmin);

    float inv_vdc = 1.0f / v_dc;
    float a = 0.5f + (va - vcm) * inv_vdc;
    float b = 0.5f + (vb - vcm) * inv_vdc;
    float c = 0.5f + (vc - vcm) * inv_vdc;

    /* clamp for overmodulation robustness */
    *da = a < 0.0f ? 0.0f : (a > 1.0f ? 1.0f : a);
    *db = b < 0.0f ? 0.0f : (b > 1.0f ? 1.0f : b);
    *dc = c < 0.0f ? 0.0f : (c > 1.0f ? 1.0f : c);
}

/* ---- current-loop step ---- */

void foc_init(foc_t *f, const foc_params_t *par)
{
    f->par = *par;
    f->pi_d.kp = par->kp_d; f->pi_d.ki = par->ki_d;
    f->pi_d.ts = par->ts;   f->pi_d.integ = 0.0f;
    f->pi_q.kp = par->kp_q; f->pi_q.ki = par->ki_q;
    f->pi_q.ts = par->ts;   f->pi_q.integ = 0.0f;
}

void foc_step(foc_t *f, const foc_in_t *in, foc_out_t *out)
{
    const foc_params_t *p = &f->par;

    float sin_th = sinf(in->theta_e);
    float cos_th = cosf(in->theta_e);

    float ialpha, ibeta;
    foc_clarke(in->ia, in->ib, in->ic, &ialpha, &ibeta);
    foc_park(ialpha, ibeta, sin_th, cos_th, &out->id, &out->iq);

    /* cross-coupling feedforward: cancels the w_e terms of the dq plant
     * so each PI sees a plain RL branch (prerequisite for the
     * modulus-optimum gain derivation) */
    float ff_d = -in->w_e * p->lq * out->iq;
    float ff_q =  in->w_e * (p->ld * out->id + p->psi_f);

    /* SVPWM linear-region voltage budget; d-axis gets priority so flux
     * control never starves, q-axis gets the remainder */
    float v_max = in->v_dc * ONE_OVER_SQRT3;

    float vd = ff_d + foc_pi_step(&f->pi_d, in->id_ref - out->id,
                                  -v_max - ff_d, v_max - ff_d);
    float vq_head = v_max * v_max - vd * vd;
    float vq_budget = sqrtf(vq_head > 0.0f ? vq_head : 0.0f);
    float vq = ff_q + foc_pi_step(&f->pi_q, in->iq_ref - out->iq,
                                  -vq_budget - ff_q, vq_budget - ff_q);

    out->sat = (vd * vd + vq * vq) >= (0.999f * v_max * v_max);
    out->vd = vd;
    out->vq = vq;

    float valpha, vbeta;
    foc_inv_park(vd, vq, sin_th, cos_th, &valpha, &vbeta);
    foc_svpwm(valpha, vbeta, in->v_dc, &out->duty_a, &out->duty_b,
              &out->duty_c);
}
