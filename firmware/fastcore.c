#include "fastcore.h"

#include <math.h>

struct fc_s {
    fc_config_t   cfg;
    pmsm_params_t motor;
    pmsm_state_t  x;
    foc_t         foc;
    spd_t         spd;
    fd_t          fd;
    long          tick;      /* free-running, drives the speed divider */
    float         iq_ref;    /* held between speed-loop updates        */
    unsigned      faults;    /* latched                                */
};

size_t fc_sizeof(void)
{
    return sizeof(struct fc_s);
}

void fc_init(fc_t *fc, const fc_config_t *cfg, const pmsm_params_t *motor,
             const foc_params_t *foc_par, const spd_params_t *spd_par,
             const fd_params_t *fd_par)
{
    fc->cfg = *cfg;
    fc->motor = *motor;
    pmsm_init(&fc->x);
    foc_init(&fc->foc, foc_par);
    spd_init(&fc->spd, spd_par);
    fd_init(&fc->fd, fd_par);
    fc->tick = 0;
    fc->iq_ref = 0.0f;
    fc->faults = 0u;
}

void fc_set_speed(fc_t *fc, double w_m)
{
    fc->x.w_m = w_m;
}

void fc_clear_faults(fc_t *fc)
{
    fc->faults = 0u;
    fc->fd.flags = 0u;
}

int fc_advance(fc_t *fc, const fc_cmd_t *cmd, int n_ticks, fc_out_t *out)
{
    const double dt_sub = fc->cfg.dt / (double)fc->cfg.n_sub;
    double sum_p = 0.0, sum_te = 0.0, sum_w = 0.0;
    double vd_applied = 0.0, vq_applied = 0.0;
    double duty[3] = {0.0, 0.0, 0.0};
    foc_out_t fo = {0};
    int sat = 0;
    int k, s;

    out->status = FC_OK;

    for (k = 0; k < n_ticks; k++) {
        double theta_e, w_e, ia, ib, ic;
        foc_in_t fi;

        /* Dyno mode: an infinitely stiff shaft holds the speed, which is how
         * the current-loop figures isolate the inner loops from mechanics. */
        if (cmd->clamp_speed) fc->x.w_m = cmd->w_clamp;

        theta_e = pmsm_theta_e(&fc->motor, &fc->x);
        w_e = pmsm_w_e(&fc->motor, &fc->x);
        pmsm_dq_to_phase(fc->x.i_d, fc->x.i_q, theta_e, &ia, &ib, &ic);

        /* Outer speed loop on its divider. The divider counts absolute ticks
         * so that a window boundary cannot shift the speed-loop phase — a
         * caller free to choose n_ticks must not be able to change the
         * control timing. */
        if (cmd->mode == FC_MODE_SPEED) {
            if (fc->tick % fc->cfg.speed_div == 0)
                fc->iq_ref = spd_step(&fc->spd, (float)cmd->w_ref,
                                      (float)fc->x.w_m);
        } else {
            fc->iq_ref = (float)cmd->iq_ref;
        }

        fi.ia = (float)ia;
        fi.ib = (float)ib;
        fi.ic = (float)ic;
        fi.theta_e = (float)theta_e;
        fi.w_e = (float)w_e;
        fi.id_ref = (cmd->mode == FC_MODE_SPEED) ? 0.0f : (float)cmd->id_ref;
        fi.iq_ref = fc->iq_ref;
        fi.v_dc = (float)fc->cfg.v_dc;

        foc_step(&fc->foc, &fi, &fo);
        sat += fo.sat;

        fc->faults |= fd_step(&fc->fd, fi.ia, fi.ib, fi.ic, fi.theta_e,
                              fi.w_e, fo.id, fo.iq);

        /* Zero-order hold: the duties stand for the whole PWM period. */
        duty[0] = fo.duty_a;
        duty[1] = fo.duty_b;
        duty[2] = fo.duty_c;
        pmsm_duties_to_vdq(duty, fc->cfg.v_dc, theta_e,
                           &vd_applied, &vq_applied);

        for (s = 0; s < fc->cfg.n_sub; s++)
            pmsm_step(&fc->motor, &fc->x, vd_applied, vq_applied,
                      cmd->t_load, dt_sub);

        if (!isfinite(fc->x.i_d) || !isfinite(fc->x.i_q) ||
            !isfinite(fc->x.w_m)) {
            out->status = FC_ERR_NONFINITE;
            break;
        }

        /* Amplitude-invariant dq, so instantaneous three-phase power carries
         * the 3/2 factor. Sampled after the substeps; across a 100 us period
         * the within-tick current change is immaterial to a window mean. */
        sum_p += 1.5 * (vd_applied * fc->x.i_d + vq_applied * fc->x.i_q);
        sum_te += pmsm_torque(&fc->motor, fc->x.i_d, fc->x.i_q);
        sum_w += fc->x.w_m;

        fc->tick++;
    }

    {
        const double n = (double)(k > 0 ? k : 1);
        out->p_elec_mean = sum_p / n;
        out->torque_mean = sum_te / n;
        out->w_m_mean = sum_w / n;
    }

    out->i_d = fc->x.i_d;
    out->i_q = fc->x.i_q;
    out->w_m = fc->x.w_m;
    out->theta_e = pmsm_theta_e(&fc->motor, &fc->x);
    out->torque = pmsm_torque(&fc->motor, fc->x.i_d, fc->x.i_q);
    out->vd = vd_applied;
    out->vq = vq_applied;
    out->iq_ref = (double)fc->iq_ref;
    out->duty_a = duty[0];
    out->duty_b = duty[1];
    out->duty_c = duty[2];
    out->fault_flags = fc->faults;
    out->sat_count = sat;

    return out->status;
}
