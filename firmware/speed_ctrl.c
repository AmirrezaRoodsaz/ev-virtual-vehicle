#include "speed_ctrl.h"

void spd_init(spd_t *s, const spd_params_t *par)
{
    s->par = *par;
    s->pi.kp = par->kp;
    s->pi.ki = par->ki;
    s->pi.ts = par->ts;
    s->pi.integ = 0.0f;
    s->w_ref_f = 0.0f;
}

float spd_step(spd_t *s, float w_ref, float w_m)
{
    const spd_params_t *p = &s->par;

    if (p->t_filt > 0.0f) {
        /* one-pole reference prefilter: cancels the symmetric-optimum
         * PI zero for reference steps (43% -> ~8% overshoot) without
         * touching the disturbance response */
        float a = p->ts / (p->t_filt + p->ts);
        s->w_ref_f += a * (w_ref - s->w_ref_f);
    } else {
        s->w_ref_f = w_ref;
    }

    return foc_pi_step(&s->pi, s->w_ref_f - w_m, -p->iq_max, p->iq_max);
}
