#include "pmsm_plant.h"

#include <math.h>

#define TWO_PI 6.283185307179586

void pmsm_init(pmsm_state_t *x)
{
    x->i_d = 0.0;
    x->i_q = 0.0;
    x->w_m = 0.0;
    x->theta_m = 0.0;
}

double pmsm_torque(const pmsm_params_t *m, double i_d, double i_q)
{
    return 1.5 * (double)m->p * (m->psi_f * i_q + (m->ld - m->lq) * i_d * i_q);
}

double pmsm_theta_e(const pmsm_params_t *m, const pmsm_state_t *x)
{
    double th = fmod((double)m->p * x->theta_m, TWO_PI);
    return th < 0.0 ? th + TWO_PI : th;
}

double pmsm_w_e(const pmsm_params_t *m, const pmsm_state_t *x)
{
    return (double)m->p * x->w_m;
}

/* dx/dt at state x. Ordered [id, iq, w_m, theta_m]. */
static void deriv(const pmsm_params_t *m, const double x[4],
                  double vd, double vq, double t_load, double dx[4])
{
    const double i_d = x[0], i_q = x[1], w_m = x[2];
    const double w_e = (double)m->p * w_m;

    dx[0] = (vd - m->rs * i_d + w_e * m->lq * i_q) / m->ld;
    dx[1] = (vq - m->rs * i_q - w_e * (m->ld * i_d + m->psi_f)) / m->lq;
    dx[2] = (pmsm_torque(m, i_d, i_q) - t_load - m->b * w_m) / m->j;
    dx[3] = w_m;
}

void pmsm_step(const pmsm_params_t *m, pmsm_state_t *x,
               double vd, double vq, double t_load, double dt)
{
    double x0[4] = {x->i_d, x->i_q, x->w_m, x->theta_m};
    double k1[4], k2[4], k3[4], k4[4], xt[4];
    int i;

    deriv(m, x0, vd, vq, t_load, k1);
    for (i = 0; i < 4; i++) xt[i] = x0[i] + 0.5 * dt * k1[i];
    deriv(m, xt, vd, vq, t_load, k2);
    for (i = 0; i < 4; i++) xt[i] = x0[i] + 0.5 * dt * k2[i];
    deriv(m, xt, vd, vq, t_load, k3);
    for (i = 0; i < 4; i++) xt[i] = x0[i] + dt * k3[i];
    deriv(m, xt, vd, vq, t_load, k4);

    for (i = 0; i < 4; i++)
        x0[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);

    x->i_d = x0[0];
    x->i_q = x0[1];
    x->w_m = x0[2];

    /* Wrap the mechanical angle. Pole pairs are an integer, so shifting
     * theta_m by 2 pi shifts theta_e by an exact multiple of 2 pi and
     * changes no observable. Without this the angle grows without bound
     * and loses absolute precision over a long real-time drive. */
    x->theta_m = fmod(x0[3], TWO_PI);
    if (x->theta_m < 0.0) x->theta_m += TWO_PI;
}

void pmsm_duties_to_vdq(const double duty[3], double v_dc, double theta_e,
                        double *vd, double *vq)
{
    const double cm = (duty[0] + duty[1] + duty[2]) / 3.0;
    const double va = (duty[0] - cm) * v_dc;
    const double vb = (duty[1] - cm) * v_dc;
    const double vc = (duty[2] - cm) * v_dc;

    const double valpha = (2.0 * va - vb - vc) / 3.0;
    const double vbeta  = (vb - vc) / 1.7320508075688772; /* sqrt(3) */

    const double c = cos(theta_e), s = sin(theta_e);
    *vd =  valpha * c + vbeta * s;
    *vq = -valpha * s + vbeta * c;
}

void pmsm_dq_to_phase(double i_d, double i_q, double theta_e,
                      double *ia, double *ib, double *ic)
{
    const double c = cos(theta_e), s = sin(theta_e);
    const double ialpha = i_d * c - i_q * s;
    const double ibeta  = i_d * s + i_q * c;

    *ia = ialpha;
    *ib = -0.5 * ialpha + 0.8660254037844386 * ibeta; /* sqrt(3)/2 */
    *ic = -0.5 * ialpha - 0.8660254037844386 * ibeta;
}
