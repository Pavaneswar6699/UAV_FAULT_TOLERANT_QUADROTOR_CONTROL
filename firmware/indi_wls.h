/* Fault-tolerant controller: position PD -> INDI on angular acceleration -> WLS motor allocation.
 *
 * Plain C port of IndiWlsController and wls_allocate in sim/control.py and sim/allocation.py.
 * No firmware dependencies, so sim/tests/test_firmware_core.py can check it against the Python version.
 *
 * Frames: world x-east, y-north, z-up; body x-forward, y-left, z-up.
 * Motors: M1 front-right, M2 back-right, M3 back-left, M4 front-left (index 0-3).
 */
#pragma once

#include <stdbool.h>

typedef struct {
  // Drone
  float mass;          // [kg]
  float fMax;          // maximum thrust of one healthy motor [N]
  float armD;          // motor offset along body x and y [m] (arm length / sqrt 2)
  float kappa;         // drag torque per newton of thrust [m]
  float inertia[3];    // [kg m^2]
  float motorTau;      // motor time constant [s]
  // Gains and limits (same names as sim/control.py)
  float kpPos;
  float kdPos;
  float kAtt;
  float kRate;
  float normalTilt;    // [rad]
  float faultTilt;     // [rad]
  float thrustMargin;
  float faultCap;      // thrust left on the faulty motor, fraction of fMax
  float yawRampS;
  float indiCutoffHz;
} IndiWlsParams;

typedef struct {
  float gyroF[3];      // filtered gyro [rad/s]
  float tauF[3];       // filtered motor torque estimate [N m]
  float fEst[4];       // motor thrust estimate [N]
  int faultMotor;      // -1 = none, 0-3
  float faultTime;     // [s]
} IndiWlsState;

typedef struct {
  float pos[3];
  float vel[3];
  float rot[9];        // body-to-world rotation, row-major
  float gyro[3];       // [rad/s]
} IndiWlsSensors;

typedef struct {
  float pos[3];
  float vel[3];
  float acc[3];        // feed-forward [m/s^2]
  float yaw;           // [rad]
} IndiWlsReference;

// sim/quad.py QuadParams defaults and sim/control.py constants; usable as a static initializer
#define INDI_WLS_DEFAULT_PARAMS {                                          \
  .mass = 0.027f, .fMax = 0.148989375f, .armD = 0.0280721392f,             \
  .kappa = 0.0251265823f, .inertia = {1.4e-5f, 1.4e-5f, 2.17e-5f},         \
  .motorTau = 0.03f, .kpPos = 2.5f, .kdPos = 2.25f, .kAtt = 4.0f,          \
  .kRate = 20.0f, .normalTilt = 0.436332313f, .faultTilt = 0.261799388f,   \
  .thrustMargin = 0.9f, .faultCap = 0.25f, .yawRampS = 0.2f,               \
  .indiCutoffHz = 30.0f }

void indiWlsDefaultParams(IndiWlsParams* p);
void indiWlsReset(IndiWlsState* s, const IndiWlsParams* p);
void indiWlsNotifyFault(IndiWlsState* s, int motor, float t);

// One control step; writes the commanded thrust of each motor [N].
void indiWlsUpdate(IndiWlsState* s, const IndiWlsParams* p, float dt, float t,
                   const IndiWlsSensors* sens, const IndiWlsReference* ref, float fCmd[4]);

// Motor thrusts closest to the wanted [T, tau_x, tau_y, tau_z], each within 0..fUpper[i], by priority.
void indiWlsAllocate(const IndiWlsParams* p, const float wrench[4], const float fUpper[4],
                     const float weights[4], float fOut[4]);
