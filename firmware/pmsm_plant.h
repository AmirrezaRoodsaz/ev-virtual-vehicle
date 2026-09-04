/*
 * pmsm_plant.h — continuous dq-frame PMSM model with fixed-step RK4,
 * plus the ideal inverter and the sensor view that surround it.
 *
 * NOT vendored — owned by this repository (see firmware/VENDORED.md).
 *
 * This is the *plant*, not controller code: it represents physics the
 * firmware acts on, and never ships to a target. It therefore breaks two
 * conventions the vendored control code follows, deliberately:
 *
 *   - double, not float. The controller uses float because a real motor
 *     MCU does; the plant is the reference physics the controller is
 *     judged against, so it should not contribute the error it is meant
 *     to measure. Over a long real-time drive the difference accumulates.
 *   - it lives in firmware/ anyway, because it compiles into the same
 *     shared library and runs inside the same 10 kHz C loop (S3). Keeping
 *     it here is what buys the real-time headroom; the file header is the
 *     boundary marker.
 *
 * State x = [id, iq, w_m, theta_m] — currents [A], mechanical speed
 * [rad/s], mechanical angle [rad]. Inputs are stator voltage in the rotor
 * dq frame and load torque at the shaft.
 *
 * Electrical dynamics (rotor reference frame, motor convention):
 *   d id/dt = (vd - Rs id + w_e Lq iq) / Ld
 *   d iq/dt = (vq - Rs iq - w_e (Ld id + psi_f)) / Lq
 *
 * Torque, alignment plus reluctance term (Ld < Lq for an IPM):
 *   Te = 1.5 p (psi_f iq + (Ld - Lq) id iq)
 *
 * Mechanics:
 *   J dw_m/dt = Te - T_load - B w_m,   w_e = p w_m
 *
 * Known limitation, carried over from edrive-foc-control and stated in the
 * README: Ld and Lq are constant, so magnetic saturation is not modelled.
 * The machine reaches ~189 N m of its cited 200 N m rating for this reason.
 */

#ifndef PMSM_PLANT_H
#define PMSM_PLANT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double rs;      /* stator phase resistance [ohm]     */
    double ld;      /* d-axis inductance [H]             */
    double lq;      /* q-axis inductance [H]             */
    double psi_f;   /* PM flux linkage [Wb]              */
    double j;       /* rotor inertia [kg m^2]            */
    double b;       /* viscous friction [N m s/rad]      */
    int    p;       /* pole pairs                        */
} pmsm_params_t;

typedef struct {
    double i_d;
    double i_q;
    double w_m;      /* mechanical speed [rad/s]                        */
    double theta_m;  /* mechanical angle [rad], wrapped to [0, 2 pi)    */
} pmsm_state_t;

/* Zero all states. */
void pmsm_init(pmsm_state_t *x);

/* Electromagnetic torque [N m] at an arbitrary operating point. */
double pmsm_torque(const pmsm_params_t *m, double i_d, double i_q);

/* Electrical angle [rad] in [0, 2 pi), and electrical speed [rad/s]. */
double pmsm_theta_e(const pmsm_params_t *m, const pmsm_state_t *x);
double pmsm_w_e(const pmsm_params_t *m, const pmsm_state_t *x);

/* One RK4 step of dt [s] with zero-order-hold inputs. */
void pmsm_step(const pmsm_params_t *m, pmsm_state_t *x,
               double vd, double vq, double t_load, double dt);

/* Ideal inverter: per-phase duty cycles -> average dq voltage over the PWM
 * period. Floating neutral, so the common mode in the duties is rejected;
 * no dead time and no switching ripple (documented limitation). */
void pmsm_duties_to_vdq(const double duty[3], double v_dc, double theta_e,
                        double *vd, double *vq);

/* What the phase-current sensors would read: inverse Park, inverse Clarke. */
void pmsm_dq_to_phase(double i_d, double i_q, double theta_e,
                      double *ia, double *ib, double *ic);

#ifdef __cplusplus
}
#endif

#endif /* PMSM_PLANT_H */
