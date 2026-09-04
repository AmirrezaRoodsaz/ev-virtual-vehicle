/*
 * fastcore.h — the 10 kHz fast domain, run entirely inside C.
 *
 * NOT vendored — owned by this repository.
 *
 * Why this file exists
 * --------------------
 * edrive-foc-control stepped the controller from Python: one ctypes call per
 * PWM period, with the plant integrated in numpy underneath. That is fine for
 * offline scenarios and far too slow for a cockpit that must hold real time —
 * at 10 kHz it costs ~10,000 boundary crossings plus thousands of scalar numpy
 * operations per simulated second.
 *
 * fastcore closes the loop inside C. Python calls fc_advance() once per
 * millisecond, and one call runs ten complete PWM periods: measure, speed loop
 * on its divider, foc_step, plausibility monitor, inverter, and n_sub RK4
 * plant substeps. The boundary is crossed 1,000 times per simulated second
 * instead of 10,000, and nothing crosses it that Python does not actually need.
 *
 * This is the physical split the project is organised around: fast electrical
 * dynamics in C, slow thermal/electrochemical/vehicle dynamics in Python. The
 * outputs below are chosen accordingly — instantaneous state for plotting, and
 * *window means* for the quantities the slow domain integrates, above all
 * electrical power, which is what the battery pack will consume from S7 on.
 *
 * The control code called here is vendored and unmodified. fastcore is a
 * scheduler, not a controller: it decides when things run, never what they do.
 */

#ifndef FASTCORE_H
#define FASTCORE_H

#include <stddef.h>

#include "fault_detect.h"
#include "foc.h"
#include "pmsm_plant.h"
#include "speed_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Command modes. */
#define FC_MODE_CURRENT 0   /* id_ref / iq_ref commanded directly (dyno)  */
#define FC_MODE_SPEED   1   /* outer speed loop generates iq_ref          */

/* fc_out_t.status */
#define FC_OK            0
#define FC_ERR_NONFINITE 1  /* plant state diverged — sim must stop       */

typedef struct {
    double v_dc;       /* DC-link voltage [V]                             */
    double dt;         /* PWM / current-ISR period [s]                    */
    int    n_sub;      /* plant RK4 substeps per PWM period               */
    int    speed_div;  /* speed loop runs every N PWM periods             */
} fc_config_t;

typedef struct {
    int    mode;        /* FC_MODE_*                                      */
    double id_ref;      /* [A] current mode; forced to 0 in speed mode    */
    double iq_ref;      /* [A] current mode                               */
    double w_ref;       /* [rad/s] mechanical, speed mode                 */
    double t_load;      /* [N m] load torque at the shaft                 */
    int    clamp_speed; /* dyno mode: hold mechanical speed at w_clamp    */
    double w_clamp;     /* [rad/s]                                        */
} fc_cmd_t;

typedef struct {
    /* instantaneous state at the end of the window */
    double i_d, i_q;
    double w_m;        /* mechanical speed [rad/s]                        */
    double theta_e;    /* electrical angle [rad]                          */
    double torque;     /* electromagnetic torque [N m]                    */
    double vd, vq;     /* dq voltage actually applied by the inverter [V] */
    double iq_ref;     /* what the speed loop asked for [A]               */
    double duty_a, duty_b, duty_c;

    /* window means — what the slow domain integrates */
    double p_elec_mean;  /* electrical power [W], motor-positive          */
    double torque_mean;  /* [N m]                                         */
    double w_m_mean;     /* [rad/s]                                       */

    unsigned fault_flags; /* latched FD_* bits from the vendored monitor  */
    int      sat_count;   /* PWM periods in this window that hit the
                             voltage limit                               */
    int      status;      /* FC_OK or FC_ERR_*                            */
} fc_out_t;

/* Opaque to Python: allocate fc_sizeof() bytes rather than mirroring the
 * struct, so adding controller state here never silently breaks the binding. */
typedef struct fc_s fc_t;

size_t fc_sizeof(void);

void fc_init(fc_t *fc, const fc_config_t *cfg, const pmsm_params_t *motor,
             const foc_params_t *foc_par, const spd_params_t *spd_par,
             const fd_params_t *fd_par);

/* Set the initial mechanical operating point (used to start a scenario at
 * speed without integrating up to it). */
void fc_set_speed(fc_t *fc, double w_m);

/* Run n_ticks complete PWM periods. Returns out->status for convenience. */
int fc_advance(fc_t *fc, const fc_cmd_t *cmd, int n_ticks, fc_out_t *out);

/* Latched faults are sticky by design; the application decides when to clear. */
void fc_clear_faults(fc_t *fc);

#ifdef __cplusplus
}
#endif

#endif /* FASTCORE_H */
