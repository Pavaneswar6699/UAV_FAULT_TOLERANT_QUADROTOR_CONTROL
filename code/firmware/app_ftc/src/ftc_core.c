/* Fault-tolerant controller core. See ftc_core.h and code/ftqc/controller.py. */
#include "ftc_core.h"

#include <math.h>
#include <string.h>

#define FTC_G 9.81f
#define FTC_PI 3.14159265f
#define FTC_FEASIBILITY_TOL 1e-6f  /* N, float rounding slack in the feasibility tests */
static const float YAW_DIR[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

static void biquadInit(ftcBiquad3_t *f, float cutoffHz, float dt) {
  const float w = tanf(FTC_PI * cutoffHz * dt);
  const float k = 1.0f / (1.0f + 1.41421356f * w + w * w);
  f->b0 = w * w * k;
  f->b1 = 2.0f * f->b0;
  f->b2 = f->b0;
  f->a1 = 2.0f * (w * w - 1.0f) * k;
  f->a2 = (1.0f - 1.41421356f * w + w * w) * k;
  f->primed = false;
}

static void biquadApply(ftcBiquad3_t *f, const float x[3], float y[3]) {
  for (int i = 0; i < 3; i++) {
    if (!f->primed) {
      f->x1[i] = f->x2[i] = f->y1[i] = f->y2[i] = x[i];
    }
    y[i] = f->b0 * x[i] + f->b1 * f->x1[i] + f->b2 * f->x2[i] - f->a1 * f->y1[i] - f->a2 * f->y2[i];
    f->x2[i] = f->x1[i];
    f->x1[i] = x[i];
    f->y2[i] = f->y1[i];
    f->y1[i] = y[i];
  }
  f->primed = true;
}

static float wrapPi(float a) {
  while (a > FTC_PI) a -= 2.0f * FTC_PI;
  while (a < -FTC_PI) a += 2.0f * FTC_PI;
  return a;
}

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* Minimum-norm forces for thrust T and roll/pitch torques, with zero yaw torque. */
static void baseSolution(const ftcPlant_t *p, float T, float tx, float ty, float f0[4]) {
  const float a = p->arm * 0.70710678f;
  f0[0] = 0.25f * (T + (-tx - ty) / a);
  f0[1] = 0.25f * (T + (-tx + ty) / a);
  f0[2] = 0.25f * (T + (tx + ty) / a);
  f0[3] = 0.25f * (T + (tx - ty) / a);
}

/* Range of lam with 0 <= f0 + lam * YAW_DIR <= upper for every motor. */
static bool lamInterval(const float f0[4], const float upper[4], float *lo, float *hi) {
  *lo = -1e9f;
  *hi = 1e9f;
  for (int i = 0; i < 4; i++) {
    const float a = (0.0f - f0[i]) / YAW_DIR[i];
    const float b = (upper[i] - f0[i]) / YAW_DIR[i];
    *lo = fmaxf(*lo, fminf(a, b));
    *hi = fminf(*hi, fmaxf(a, b));
  }
  return *lo <= *hi;
}

/* Exact range [lo, hi] of collective thrust that fits the motor limits for these roll/pitch
 * torques with yaw left free (see thrust_interval in allocation.py). False if no thrust fits. */
static bool thrustInterval(const ftcPlant_t *p, float tx, float ty, const float upper[4], float *lo, float *hi) {
  float d[4], low0[4], lowSlope[4], up0[4], upSlope[4];
  baseSolution(p, 0.0f, tx, ty, d);
  for (int i = 0; i < 4; i++) {
    const bool spinsUp = YAW_DIR[i] > 0.0f;
    low0[i] = spinsUp ? -d[i] : d[i] - upper[i];
    lowSlope[i] = spinsUp ? -0.25f : 0.25f;
    up0[i] = spinsUp ? upper[i] - d[i] : d[i];
    upSlope[i] = spinsUp ? -0.25f : 0.25f;
  }
  *lo = -1e9f;
  *hi = 1e9f;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      const float c = low0[i] - up0[j];       /* need c + s * T <= 0 */
      const float s = lowSlope[i] - upSlope[j];
      if (s > 0.0f) {
        *hi = fminf(*hi, -c / s);
      } else if (s < 0.0f) {
        *lo = fmaxf(*lo, -c / s);
      } else if (c > FTC_FEASIBILITY_TOL) {
        return false;
      }
    }
  }
  if (*lo > *hi + FTC_FEASIBILITY_TOL) return false;
  *hi = fmaxf(*hi, *lo);
  return true;
}

/* Priority: roll/pitch torque > thrust > yaw torque (see allocation.py). */
bool ftcMix(const ftcPlant_t *p, const float v[4], const float upper[4], float f[4], bool *yawDropped) {
  static const float attScales[5] = {1.0f, 0.75f, 0.5f, 0.25f, 0.0f};
  const float T = v[0], tx = v[1], ty = v[2];
  const float lamYaw = v[3] / (4.0f * p->kTorque);

  for (int s = 0; s < 5; s++) {
    float tLo, tHi, f0[4], lo, hi;
    if (!thrustInterval(p, attScales[s] * tx, attScales[s] * ty, upper, &tLo, &tHi)) continue;
    baseSolution(p, clampf(T, tLo, tHi), attScales[s] * tx, attScales[s] * ty, f0);
    (void)lamInterval(f0, upper, &lo, &hi);
    const float lam = fminf(fmaxf(lamYaw, lo), hi);
    for (int i = 0; i < 4; i++) {
      f[i] = clampf(f0[i] + lam * YAW_DIR[i], 0.0f, upper[i]);
    }
    *yawDropped = (lamYaw < lo) || (lamYaw > hi);
    return true;
  }
  for (int i = 0; i < 4; i++) f[i] = 0.0f;
  *yawDropped = true;
  return false;
}

void ftcInit(ftcState_t *s, const ftcPlant_t *p, const ftcGains_t *g) {
  memset(s, 0, sizeof(*s));
  s->p = *p;
  s->g = *g;
  biquadInit(&s->gyroLpf, g->lpfHz, g->dt);
  biquadInit(&s->tauLpf, g->lpfHz, g->dt);
  for (int i = 0; i < 4; i++) s->fEst[i] = p->mass * FTC_G / 4.0f;
  s->faultCap = 0.25f;
}

void ftcStep(ftcState_t *s, const ftcMeas_t *m, const ftcSetpoint_t *sp, float fOut[4]) {
  const ftcPlant_t *p = &s->p;
  const ftcGains_t *g = &s->g;
  const bool faulted = s->faultMotor >= 1 && s->faultMotor <= 4;

  /* 1. desired acceleration -> thrust vector (world) */
  float a[3];
  for (int i = 0; i < 3; i++) {
    const float kp = (i < 2) ? g->kpXY : g->kpZ;
    const float kd = (i < 2) ? g->kdXY : g->kdZ;
    a[i] = sp->acc[i] + kp * (sp->pos[i] - m->pos[i]) + kd * (sp->vel[i] - m->vel[i]);
  }
  const float axyMax = fmaxf(a[2] + FTC_G, 0.0f) * tanf(g->tiltMax);
  const float nxy = sqrtf(a[0] * a[0] + a[1] * a[1]);
  if (nxy > axyMax) {
    a[0] *= axyMax / fmaxf(nxy, 1e-9f);
    a[1] *= axyMax / fmaxf(nxy, 1e-9f);
  }
  float t[3] = {p->mass * a[0], p->mass * a[1], p->mass * (a[2] + FTC_G)};
  t[2] = fmaxf(t[2], 0.2f * p->mass * FTC_G);
  const float zb[3] = {m->R[0][2], m->R[1][2], m->R[2][2]};
  const float T = fmaxf(t[0] * zb[0] + t[1] * zb[1] + t[2] * zb[2], 0.0f);
  const float tn = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
  const float zd[3] = {t[0] / tn, t[1] / tn, t[2] / tn};

  /* 2. yaw-independent tilt error (zb x zd, in body frame) -> body-rate command */
  const float c[3] = {zb[1] * zd[2] - zb[2] * zd[1], zb[2] * zd[0] - zb[0] * zd[2], zb[0] * zd[1] - zb[1] * zd[0]};
  float eb[3];
  for (int i = 0; i < 3; i++) eb[i] = m->R[0][i] * c[0] + m->R[1][i] * c[1] + m->R[2][i] * c[2];
  const float yaw = atan2f(m->R[1][0], m->R[0][0]);
  const float wDes[3] = {g->kAtt * eb[0], g->kAtt * eb[1], faulted ? 0.0f : g->kYaw * wrapPi(sp->yaw - yaw)};

  /* 3. INDI rate loop on the filtered gyro and filtered actuator torque estimate */
  float wf[3], dw[3] = {0.0f, 0.0f, 0.0f}, tauEst[3], tauF[3];
  biquadApply(&s->gyroLpf, m->gyro, wf);
  if (s->havePrev) {
    for (int i = 0; i < 3; i++) dw[i] = (wf[i] - s->wPrev[i]) / g->dt;
  }
  memcpy(s->wPrev, wf, sizeof(wf));
  s->havePrev = true;
  const float ar = p->arm * 0.70710678f;
  const float *fe = s->fEst;
  tauEst[0] = ar * (-fe[0] - fe[1] + fe[2] + fe[3]);
  tauEst[1] = ar * (-fe[0] + fe[1] + fe[2] - fe[3]);
  tauEst[2] = p->kTorque * (-fe[0] + fe[1] - fe[2] + fe[3]);
  biquadApply(&s->tauLpf, tauEst, tauF);
  float tau[3];
  for (int i = 0; i < 3; i++) tau[i] = tauF[i] + p->J[i] * (g->kRate * (wDes[i] - wf[i]) - dw[i]);

  if (faulted) {
    tau[2] = 0.0f;  /* no yaw torque wanted; the mixer drops yaw */
    /* spin compensation: pre-rotate roll/pitch torque by the turn during the motor lag */
    const float psi = -m->gyro[2] * g->leadTime;
    const float cs = cosf(psi), sn = sinf(psi);
    const float t0 = cs * tau[0] - sn * tau[1];
    const float t1 = sn * tau[0] + cs * tau[1];
    tau[0] = t0;
    tau[1] = t1;
  }

  /* 4. mixing within per-motor limits */
  const float v[4] = {T, tau[0], tau[1], tau[2]};
  float upper[4] = {p->fMax, p->fMax, p->fMax, p->fMax};
  if (faulted) upper[s->faultMotor - 1] = s->faultCap * p->fMax;
  ftcMix(p, v, upper, fOut, &s->yawDropped);

  for (int i = 0; i < 4; i++) {
    s->fEst[i] += (g->dt / p->tauMotor) * (fOut[i] - s->fEst[i]);
    s->f[i] = fOut[i];
  }
  s->thrust = T;
  memcpy(s->tau, tau, sizeof(tau));
}
