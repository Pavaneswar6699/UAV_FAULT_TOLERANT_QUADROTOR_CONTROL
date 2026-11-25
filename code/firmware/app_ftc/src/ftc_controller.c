/* Crazyflie out-of-tree controller: glue between the firmware and ftc_core.
 * Select it in flight with the parameter stabilizer.controller = 6 (ControllerTypeOot).
 *
 * Parameters (group "ftc"):
 *   faultMotor  0 = healthy, 1..4 = the controller is told this motor is weak (fault notification)
 *   faultCap    thrust fraction left on the weak motor (0.25 for 50% RPM, since thrust ~ RPM^2)
 *   injMotor    TEST ONLY: emulate the external 50% limit on this motor (0 = off)
 *   injCap      TEST ONLY: thrust fraction allowed on injMotor (calibrate so the tachometer reads 50% RPM)
 *   gains: kpXY kdXY kpZ kdZ kAtt kRate leadTime
 * Logs (group "ftc"): f1..f4 [N], thrust [N], mode (0 healthy, 1 fault mode, 2 fault + yaw dropped)
 */
#include <math.h>
#include <string.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FTC"
#include "debug.h"

#include "controller.h"
#include "log.h"
#include "param.h"
#include "platform_defaults.h"
#include "stabilizer_types.h"

#include "ftc_core.h"

static ftcState_t ftc;
static ftcGains_t gains = {
  .kpXY = 4.0f, .kdXY = 3.0f, .kpZ = 6.0f, .kdZ = 4.0f, .tiltMax = 0.436f,
  .kAtt = 8.0f, .kYaw = 2.0f, .kRate = 25.0f, .lpfHz = 30.0f, .dt = 0.002f, .leadTime = 0.035f,
};

static uint8_t faultMotor = 0;
static float faultCap = 0.25f;
static uint8_t injMotor = 0;
static float injCap = 0.25f;

static float logF[4];
static float logThrust;
static uint8_t logMode;

void appMain(void) {
  DEBUG_PRINT("FTC app ready. Set stabilizer.controller=6 to use it.\n");
  while (1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOutOfTreeInit(void) {
  const ftcPlant_t plant = {
    .mass = CF_MASS, .arm = ARM_LENGTH, .fMax = THRUST_MAX, .kTorque = THRUST2TORQUE,
    .J = {1.66e-5f, 1.66e-5f, 2.93e-5f}, .tauMotor = 0.03f,
  };
  ftcInit(&ftc, &plant, &gains);
}

bool controllerOutOfTreeTest(void) {
  return true;
}

static void quatToR(const quaternion_t *q, float R[3][3]) {
  const float x = q->x, y = q->y, z = q->z, w = q->w;
  R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - z * w);     R[0][2] = 2 * (x * z + y * w);
  R[1][0] = 2 * (x * y + z * w);     R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - x * w);
  R[2][0] = 2 * (x * z - y * w);     R[2][1] = 2 * (y * z + x * w);     R[2][2] = 1 - 2 * (x * x + y * y);
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors,
                         const state_t *state, const stabilizerStep_t stabilizerStep) {
  control->controlMode = controlModeForce;
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;  /* keep the last output between 500 Hz updates */
  }

  if (setpoint->mode.z == modeDisable) {
    /* no position setpoint (e.g. stop setpoint / kill): motors off and controller reset */
    for (int i = 0; i < 4; i++) control->normalizedForces[i] = 0.0f;
    controllerOutOfTreeInit();
    logMode = 0;
    return;
  }

  ftc.g = gains;                                    /* allow live tuning through params */
  ftc.faultMotor = (faultMotor <= 4) ? faultMotor : 0;
  ftc.faultCap = faultCap;

  ftcMeas_t m;
  m.pos[0] = state->position.x; m.pos[1] = state->position.y; m.pos[2] = state->position.z;
  m.vel[0] = state->velocity.x; m.vel[1] = state->velocity.y; m.vel[2] = state->velocity.z;
  quatToR(&state->attitudeQuaternion, m.R);
  const float d2r = 3.14159265f / 180.0f;
  m.gyro[0] = sensors->gyro.x * d2r; m.gyro[1] = sensors->gyro.y * d2r; m.gyro[2] = sensors->gyro.z * d2r;

  ftcSetpoint_t sp;
  sp.pos[0] = setpoint->position.x; sp.pos[1] = setpoint->position.y; sp.pos[2] = setpoint->position.z;
  sp.vel[0] = setpoint->velocity.x; sp.vel[1] = setpoint->velocity.y; sp.vel[2] = setpoint->velocity.z;
  sp.acc[0] = setpoint->acceleration.x; sp.acc[1] = setpoint->acceleration.y; sp.acc[2] = setpoint->acceleration.z;
  if (setpoint->mode.quat == modeAbs) {
    const quaternion_t *q = &setpoint->attitudeQuaternion;
    sp.yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z));
  } else {
    sp.yaw = setpoint->attitude.yaw * d2r;
  }

  float f[4];
  ftcStep(&ftc, &m, &sp, f);

  for (int i = 0; i < 4; i++) {
    float n = f[i] / THRUST_MAX;
    if (injMotor >= 1 && injMotor <= 4 && i == injMotor - 1 && n > injCap) {
      n = injCap;  /* emulated external fault, for testing only */
    }
    control->normalizedForces[i] = fminf(fmaxf(n, 0.0f), 1.0f);
    logF[i] = f[i];
  }
  logThrust = ftc.thrust;
  logMode = ftc.faultMotor ? (ftc.yawDropped ? 2 : 1) : 0;
}

PARAM_GROUP_START(ftc)
PARAM_ADD(PARAM_UINT8, faultMotor, &faultMotor)
PARAM_ADD(PARAM_FLOAT, faultCap, &faultCap)
PARAM_ADD(PARAM_UINT8, injMotor, &injMotor)
PARAM_ADD(PARAM_FLOAT, injCap, &injCap)
PARAM_ADD(PARAM_FLOAT, kpXY, &gains.kpXY)
PARAM_ADD(PARAM_FLOAT, kdXY, &gains.kdXY)
PARAM_ADD(PARAM_FLOAT, kpZ, &gains.kpZ)
PARAM_ADD(PARAM_FLOAT, kdZ, &gains.kdZ)
PARAM_ADD(PARAM_FLOAT, kAtt, &gains.kAtt)
PARAM_ADD(PARAM_FLOAT, kRate, &gains.kRate)
PARAM_ADD(PARAM_FLOAT, leadTime, &gains.leadTime)
PARAM_GROUP_STOP(ftc)

LOG_GROUP_START(ftc)
LOG_ADD(LOG_FLOAT, f1, &logF[0])
LOG_ADD(LOG_FLOAT, f2, &logF[1])
LOG_ADD(LOG_FLOAT, f3, &logF[2])
LOG_ADD(LOG_FLOAT, f4, &logF[3])
LOG_ADD(LOG_FLOAT, thrust, &logThrust)
LOG_ADD(LOG_UINT8, mode, &logMode)
LOG_GROUP_STOP(ftc)
