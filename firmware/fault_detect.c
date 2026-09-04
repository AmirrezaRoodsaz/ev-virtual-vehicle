#include "fault_detect.h"

#include <math.h>

#define TWO_PI 6.28318530717958648f

void fd_init(fd_t *fd, const fd_params_t *par)
{
    fd->par = *par;
    fd->theta_prev = 0.0f;
    fd->have_prev = 0;
    fd->cnt_sum = fd->cnt_mag = fd->cnt_enc = 0;
    fd->flags = 0u;
}

/* debounce helper: count consecutive violations, latch after N */
static void check(int violated, int *cnt, unsigned bit, unsigned *flags,
                  int debounce)
{
    if (violated) {
        if (++*cnt >= debounce) *flags |= bit;
    } else {
        *cnt = 0;
    }
}

unsigned fd_step(fd_t *fd, float ia, float ib, float ic,
                 float theta_e, float w_e, float id, float iq)
{
    const fd_params_t *p = &fd->par;

    /* KCL: isolated neutral forces the three phase currents to sum to
     * zero; any single-sensor offset shows up here directly */
    float sum = ia + ib + ic;
    check(fabsf(sum) > p->sum_thresh, &fd->cnt_sum, FD_CURRENT_SUM,
          &fd->flags, p->debounce);

    /* current-vector magnitude plausibility */
    float mag = sqrtf(id * id + iq * iq);
    check(mag > p->mag_thresh, &fd->cnt_mag, FD_CURRENT_MAG,
          &fd->flags, p->debounce);

    /* encoder: measured angle increment vs the one w_e implies */
    if (fd->have_prev) {
        float dth = theta_e - fd->theta_prev;
        while (dth > 3.14159265f) dth -= TWO_PI;
        while (dth < -3.14159265f) dth += TWO_PI;
        check(fabsf(dth - w_e * p->ts) > p->ang_thresh, &fd->cnt_enc,
              FD_ENCODER, &fd->flags, p->debounce);
    }
    fd->theta_prev = theta_e;
    fd->have_prev = 1;

    return fd->flags;
}
