/* See indi_wls.h. Comments name the matching Python function. */
#include "indi_wls.h"

#include <math.h>
#include <string.h>

#define GRAVITY 9.81f
#define PI_F 3.14159265f
#define REGULARISATION 1e-4f
#define FAULT_YAW_WEIGHT 1e-3f

static const float NORMAL_WEIGHTS[4] = {10.0f, 100.0f, 100.0f, 1.0f};   // thrust, roll, pitch, yaw

void indiWlsDefaultParams(IndiWlsParams* p) {
  const IndiWlsParams defaults = INDI_WLS_DEFAULT_PARAMS;
  *p = defaults;
}

// quad.py allocation_matrix: [T, tau_x, tau_y, tau_z] = B f
static void allocationMatrix(const IndiWlsParams* p, float b[4][4]) {
  const float d = p->armD, k = p->kappa;
  const float rows[4][4] = {
    {1.0f, 1.0f, 1.0f, 1.0f},
    {-d, -d, d, d},
    {-d, d, d, -d},
    {-k, k, -k, k},
  };
  memcpy(b, rows, sizeof(rows));
}

void indiWlsReset(IndiWlsState* s, const IndiWlsParams* p) {
  float b[4][4];
  allocationMatrix(p, b);
  const float hover = p->mass * GRAVITY / 4.0f;
  for (int i = 0; i < 4; i++) {
    s->fEst[i] = hover;
  }
  for (int i = 0; i < 3; i++) {
    s->gyroF[i] = 0.0f;
    s->tauF[i] = hover * (b[i + 1][0] + b[i + 1][1] + b[i + 1][2] + b[i + 1][3]);
  }
  s->faultMotor = -1;
  s->faultTime = 0.0f;
}

void indiWlsNotifyFault(IndiWlsState* s, int motor, float t) {
  s->faultMotor = motor;
  s->faultTime = t;
}

static void cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static float norm3(const float a[3]) {
  return sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

static float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// control.py thrust_vector: position PD -> desired thrust direction (world) and collective thrust [N]
static float thrustVector(const IndiWlsSensors* sens, const IndiWlsReference* ref, const IndiWlsParams* p,
                          float maxTilt, float thrustMax, float nDes[3]) {
  float force[3];
  for (int i = 0; i < 3; i++) {
    const float aCmd = p->kpPos * (ref->pos[i] - sens->pos[i]) + p->kdPos * (ref->vel[i] - sens->vel[i]) + ref->acc[i];
    force[i] = p->mass * aCmd;
  }
  force[2] += p->mass * GRAVITY;
  force[2] = fmaxf(force[2], 0.3f * p->mass * GRAVITY);
  const float limit = force[2] * tanf(maxTilt);
  const float horiz = sqrtf(force[0] * force[0] + force[1] * force[1]);
  if (horiz > limit) {
    force[0] *= limit / horiz;
    force[1] *= limit / horiz;
  }
  const float n = norm3(force);
  for (int i = 0; i < 3; i++) {
    nDes[i] = force[i] / n;
  }
  const float* r = sens->rot;
  const float thrust = force[0] * r[2] + force[1] * r[5] + force[2] * r[8];
  return clampf(thrust, 0.0f, thrustMax);
}

// control.py full_attitude_rates: normal mode, reach the thrust direction and the heading
static void fullAttitudeRates(const float rot[9], const float nDes[3], float yawDes, float kAtt, float wDes[3]) {
  const float xC[3] = {cosf(yawDes), sinf(yawDes), 0.0f};
  float yB[3], xB[3];
  cross(nDes, xC, yB);
  const float n = norm3(yB);
  for (int i = 0; i < 3; i++) {
    yB[i] /= n;
  }
  cross(yB, nDes, xB);
  // rDes columns: xB, yB, nDes
  float rDes[9];
  for (int i = 0; i < 3; i++) {
    rDes[3 * i + 0] = xB[i];
    rDes[3 * i + 1] = yB[i];
    rDes[3 * i + 2] = nDes[i];
  }
  // e = rDes^T rot - rot^T rDes; only the off-diagonal terms are needed
  float m[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 3; k++) {
        sum += rDes[3 * k + i] * rot[3 * k + j];
      }
      m[3 * i + j] = sum;
    }
  }
  // (rot^T rDes) = m^T, so e[i][j] = m[i][j] - m[j][i]
  wDes[0] = -kAtt * 0.5f * (m[3 * 2 + 1] - m[3 * 1 + 2]);
  wDes[1] = -kAtt * 0.5f * (m[3 * 0 + 2] - m[3 * 2 + 0]);
  wDes[2] = -kAtt * 0.5f * (m[3 * 1 + 0] - m[3 * 0 + 1]);
}

// control.py reduced_attitude_rates: fault mode, only steer the thrust direction; heading is left free
static void reducedAttitudeRates(const float rot[9], const float nDes[3], float kAtt, float wDes[3]) {
  const float zB[3] = {rot[2], rot[5], rot[8]};
  float c[3];
  cross(zB, nDes, c);
  // e = rot^T c
  const float e0 = rot[0] * c[0] + rot[3] * c[1] + rot[6] * c[2];
  const float e1 = rot[1] * c[0] + rot[4] * c[1] + rot[7] * c[2];
  wDes[0] = kAtt * e0;
  wDes[1] = kAtt * e1;
  wDes[2] = 0.0f;
}

void indiWlsUpdate(IndiWlsState* s, const IndiWlsParams* p, float dt, float t,
                   const IndiWlsSensors* sens, const IndiWlsReference* ref, float fCmd[4]) {
  const bool faulted = s->faultMotor >= 0;
  float upper[4];
  float ramp = 0.0f;
  for (int i = 0; i < 4; i++) {
    upper[i] = (faulted && i == s->faultMotor) ? p->faultCap * p->fMax : p->fMax;
  }
  if (faulted) {
    ramp = fminf(1.0f, (t - s->faultTime) / p->yawRampS);
  }
  // After the fault the weak diagonal pair is capped, so total thrust is about 2.5 motors' worth.
  const float thrustMax = p->thrustMargin * (faulted ? 2.0f * (1.0f + p->faultCap) : 4.0f) * p->fMax;
  float nDes[3], wDes[3];
  const float thrust = thrustVector(sens, ref, p, faulted ? p->faultTilt : p->normalTilt, thrustMax, nDes);
  if (faulted) {
    reducedAttitudeRates(sens->rot, nDes, p->kAtt, wDes);
  } else {
    fullAttitudeRates(sens->rot, nDes, ref->yaw, p->kAtt, wDes);
  }

  // INDI: filter gyro and the motor-torque estimate the same way, then correct the difference.
  float b[4][4];
  allocationMatrix(p, b);
  const float alpha = dt / (dt + 1.0f / (2.0f * PI_F * p->indiCutoffHz));
  float wrench[4];
  wrench[0] = thrust;
  for (int i = 0; i < 3; i++) {
    const float nu = p->kRate * (wDes[i] - sens->gyro[i]);
    const float gyroF = s->gyroF[i] + alpha * (sens->gyro[i] - s->gyroF[i]);
    const float gyroDotF = (gyroF - s->gyroF[i]) / dt;
    s->gyroF[i] = gyroF;
    float tauEst = 0.0f;
    for (int j = 0; j < 4; j++) {
      tauEst += b[i + 1][j] * s->fEst[j];
    }
    s->tauF[i] += alpha * (tauEst - s->tauF[i]);
    wrench[i + 1] = s->tauF[i] + p->inertia[i] * (nu - gyroDotF);
  }
  // A spinning body sees the roll/pitch torque it needs turn at the spin rate, and the motors
  // (a first-order lag) deliver it late and weaker: lead it by that phase and gain.
  // No effect when not spinning.
  const float lag = sens->gyro[2] * p->motorTau;
  const float phase = atanf(lag);
  const float gain = sqrtf(1.0f + lag * lag);
  const float c = cosf(phase), sn = -sinf(phase);
  const float tauX = wrench[1], tauY = wrench[2];
  wrench[1] = gain * (c * tauX - sn * tauY);
  wrench[2] = gain * (sn * tauX + c * tauY);

  const float yawWeight = (1.0f - ramp) + ramp * FAULT_YAW_WEIGHT;
  const float weights[4] = {NORMAL_WEIGHTS[0], NORMAL_WEIGHTS[1], NORMAL_WEIGHTS[2], NORMAL_WEIGHTS[3] * yawWeight};
  indiWlsAllocate(p, wrench, upper, weights, fCmd);
  // Motor model, clamped at the known cap so INDI does not credit motor k with thrust it can't give.
  for (int i = 0; i < 4; i++) {
    s->fEst[i] += (dt / p->motorTau) * (clampf(fCmd[i], 0.0f, upper[i]) - s->fEst[i]);
  }
}

/* allocation.py wls_allocate:
 *   min ||W (B f - wrench)||^2 + r ||f - f_pref||^2  subject to  0 <= f <= f_upper
 * With 4 motors every motor is either at its lower bound, at its upper bound, or free: 81 cases.
 * For each case solve the least-squares problem in the free motors; the best case whose free motors
 * stay within bounds is the optimum (the problem is strictly convex). Free motors are solved with a
 * Householder QR on the 8-row stacked system, which keeps float precision even with tiny weights.
 */
#define ROWS 8

static float stackedCost(const float a[ROWS][4], const float rhs[ROWS], const float x[4]) {
  float cost = 0.0f;
  for (int i = 0; i < ROWS; i++) {
    float r = -rhs[i];
    for (int j = 0; j < 4; j++) {
      r += a[i][j] * x[j];
    }
    cost += r * r;
  }
  return cost;
}

// Least squares over the columns in `cols` (n of them); returns false if singular.
static bool solveFree(const float a[ROWS][4], const float rhs[ROWS], const int cols[4], int n, float xFree[4]) {
  float m[ROWS][4];
  float y[ROWS];
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < n; j++) {
      m[i][j] = a[i][cols[j]];
    }
    y[i] = rhs[i];
  }
  for (int k = 0; k < n; k++) {
    float normSq = 0.0f;
    for (int i = k; i < ROWS; i++) {
      normSq += m[i][k] * m[i][k];
    }
    const float alpha = (m[k][k] > 0.0f ? -1.0f : 1.0f) * sqrtf(normSq);
    if (alpha == 0.0f) {
      return false;
    }
    float v[ROWS];
    for (int i = 0; i < ROWS; i++) {
      v[i] = i < k ? 0.0f : m[i][k];
    }
    v[k] -= alpha;
    float vNormSq = 0.0f;
    for (int i = k; i < ROWS; i++) {
      vNormSq += v[i] * v[i];
    }
    if (vNormSq > 0.0f) {
      for (int j = k; j < n; j++) {
        float dot = 0.0f;
        for (int i = k; i < ROWS; i++) {
          dot += v[i] * m[i][j];
        }
        for (int i = k; i < ROWS; i++) {
          m[i][j] -= 2.0f * v[i] * dot / vNormSq;
        }
      }
      float dot = 0.0f;
      for (int i = k; i < ROWS; i++) {
        dot += v[i] * y[i];
      }
      for (int i = k; i < ROWS; i++) {
        y[i] -= 2.0f * v[i] * dot / vNormSq;
      }
    }
  }
  for (int k = n - 1; k >= 0; k--) {
    float sum = y[k];
    for (int j = k + 1; j < n; j++) {
      sum -= m[k][j] * xFree[j];
    }
    if (m[k][k] == 0.0f) {
      return false;
    }
    xFree[k] = sum / m[k][k];
  }
  return true;
}

void indiWlsAllocate(const IndiWlsParams* p, const float wrench[4], const float fUpper[4],
                     const float weights[4], float fOut[4]) {
  float b[4][4];
  allocationMatrix(p, b);
  // Rows are scaled by each axis's full authority, so the weights express priority, not units.
  // Unknowns are x = f / fMax.
  float a[ROWS][4];
  float rhs[ROWS];
  const float r = sqrtf(REGULARISATION);
  const float xPref = wrench[0] / (4.0f * p->fMax);
  for (int i = 0; i < 4; i++) {
    const float rowScale = (fabsf(b[i][0]) + fabsf(b[i][1]) + fabsf(b[i][2]) + fabsf(b[i][3])) * p->fMax;
    const float w = weights[i] / rowScale;
    for (int j = 0; j < 4; j++) {
      a[i][j] = w * b[i][j] * p->fMax;
      a[4 + i][j] = i == j ? r : 0.0f;
    }
    rhs[i] = w * wrench[i];
    rhs[4 + i] = r * xPref;
  }
  float upper[4];
  for (int i = 0; i < 4; i++) {
    upper[i] = fUpper[i] / p->fMax;
  }

  float best[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float bestCost = INFINITY;
  const float tol = 1e-6f;
  for (int pattern = 0; pattern < 81; pattern++) {
    // Each motor: 0 = at lower bound, 1 = at upper bound, 2 = free
    int code = pattern;
    int kind[4];
    int cols[4];
    int n = 0;
    float x[4];
    for (int j = 0; j < 4; j++) {
      kind[j] = code % 3;
      code /= 3;
      x[j] = kind[j] == 1 ? upper[j] : 0.0f;
      if (kind[j] == 2) {
        cols[n++] = j;
      }
    }
    if (n > 0) {
      float reduced[ROWS];
      for (int i = 0; i < ROWS; i++) {
        reduced[i] = rhs[i];
        for (int j = 0; j < 4; j++) {
          if (kind[j] != 2) {
            reduced[i] -= a[i][j] * x[j];
          }
        }
      }
      float xFree[4];
      if (!solveFree(a, reduced, cols, n, xFree)) {
        continue;
      }
      bool inside = true;
      for (int j = 0; j < n; j++) {
        if (xFree[j] < -tol || xFree[j] > upper[cols[j]] + tol) {
          inside = false;
        }
        x[cols[j]] = clampf(xFree[j], 0.0f, upper[cols[j]]);
      }
      if (!inside) {
        continue;
      }
    }
    const float cost = stackedCost(a, rhs, x);
    if (cost < bestCost) {
      bestCost = cost;
      memcpy(best, x, sizeof(best));
    }
  }
  for (int i = 0; i < 4; i++) {
    fOut[i] = best[i] * p->fMax;
  }
}
