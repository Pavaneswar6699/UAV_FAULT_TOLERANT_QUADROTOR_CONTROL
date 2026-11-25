/* Fault-tolerant controller core (pure C, no firmware headers, so it also builds on a PC).
 * C twin of code/ftqc/controller.py and code/ftqc/allocation.py. Keep them in step. */
#ifndef FTC_CORE_H
#define FTC_CORE_H

#include <stdbool.h>

typedef struct {
  float mass, arm, fMax, kTorque;   /* kg, m (motor to centre), N per motor, m (THRUST2TORQUE) */
  float J[3];                       /* kg m^2 */
  float tauMotor;                   /* s */
} ftcPlant_t;

typedef struct {
  float kpXY, kdXY, kpZ, kdZ, tiltMax;  /* tiltMax in rad */
  float kAtt, kYaw, kRate, lpfHz, dt, leadTime;
} ftcGains_t;

typedef struct {
  float b0, b1, b2, a1, a2;
  float x1[3], x2[3], y1[3], y2[3];
  bool primed;
} ftcBiquad3_t;

typedef struct {
  ftcPlant_t p;
  ftcGains_t g;
  ftcBiquad3_t gyroLpf, tauLpf;
  float wPrev[3];
  bool havePrev;
  float fEst[4];
  int faultMotor;      /* 1..4 once notified, 0 = healthy */
  float faultCap;      /* thrust fraction left on the weak motor (0.25 for 50% RPM) */
  /* last outputs, for logging */
  float f[4], thrust, tau[3];
  bool yawDropped;
} ftcState_t;

typedef struct { float pos[3], vel[3], R[3][3], gyro[3]; } ftcMeas_t;   /* world pos/vel, body gyro rad/s */
typedef struct { float pos[3], vel[3], acc[3], yaw; } ftcSetpoint_t;    /* world frame, yaw in rad */

void ftcInit(ftcState_t *s, const ftcPlant_t *p, const ftcGains_t *g);
void ftcStep(ftcState_t *s, const ftcMeas_t *m, const ftcSetpoint_t *sp, float fOut[4]);
bool ftcMix(const ftcPlant_t *p, const float v[4], const float upper[4], float f[4], bool *yawDropped);

#endif
