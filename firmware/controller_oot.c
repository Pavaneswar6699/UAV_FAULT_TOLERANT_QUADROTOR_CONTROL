/* Crazyflie out-of-tree controller slot running indi_wls.c, in CrazySim and on the real drone.
 *
 * Select it with stabilizer.controller = indiwls.controllerId (5 in CrazySim's firmware, 6 in 2026.08).
 * Runs at 500 Hz like the Python sim and sends per-motor forces (controlModeForce) as a fraction of fMax:
 * in CrazySim that fraction is the PWM; on the real drone the firmware turns it into PWM with its
 * battery-compensated thrust curve, whose full scale is THRUST_MAX, so fMax must stay THRUST_MAX there.
 * The drone's parameters are firmware parameters (group indiwls); sim/crazysim/rectangle_cflib.py sets them.
 * The fault message is indiwls.faultMotor (0 = none, 1-4), the firmware version of notify_fault().
 */
#include <math.h>

#include "controller.h"
#include "log.h"
#include "param.h"

#include "indi_wls.h"

#ifndef CONFIG_PLATFORM_SITL
#include "FreeRTOS.h"
#include "task.h"
#include "platform_defaults.h"
#endif

#define DEG_TO_RAD (3.14159265f / 180.0f)
#define CONTROL_HZ RATE_500_HZ

static IndiWlsParams params = INDI_WLS_DEFAULT_PARAMS;
static IndiWlsState state;
static uint8_t faultMotor = 0;     // parameter: what the controller has been told
static float forces[4];            // last command [N], for the log
static uint8_t controllerId = ControllerTypeOot;

#ifndef CONFIG_PLATFORM_SITL
// The app layer needs an appMain. It runs once at boot, before any client can set parameters:
// start from this drone's constants (Crazyflie 2.1 with 2.1+ propellers, as CrazySim's cf2x_P250).
void appMain(void) {
  params.mass = CF_MASS;
  params.fMax = THRUST_MAX;
  params.armD = ARM_LENGTH * 0.70710678f;
  params.kappa = THRUST2TORQUE;
  params.inertia[0] = 1.68e-5f;
  params.inertia[1] = 1.68e-5f;
  params.inertia[2] = 2.98e-5f;
  params.motorTau = 0.13f;   // drone-models identification; measure it
  while (1) {
    vTaskDelay(M2T(2000));
  }
}
#endif

void controllerOutOfTreeInit(void) {
  // Called when stabilizer.controller switches to us: keep the parameters already set
  indiWlsReset(&state, &params);
}

bool controllerOutOfTreeTest(void) {
  return true;
}

static void quatToRot(const quaternion_t* q, float r[9]) {
  const float x = q->x, y = q->y, z = q->z, w = q->w;
  r[0] = 1 - 2 * (y * y + z * z); r[1] = 2 * (x * y - w * z);     r[2] = 2 * (x * z + w * y);
  r[3] = 2 * (x * y + w * z);     r[4] = 1 - 2 * (x * x + z * z); r[5] = 2 * (y * z - w * x);
  r[6] = 2 * (x * z - w * y);     r[7] = 2 * (y * z + w * x);     r[8] = 1 - 2 * (x * x + y * y);
}

void controllerOutOfTree(control_t* control, const setpoint_t* setpoint, const sensorData_t* sensors,
                         const state_t* st, const stabilizerStep_t stabilizerStep) {
  control->controlMode = controlModeForce;
  if (!RATE_DO_EXECUTE(CONTROL_HZ, stabilizerStep)) {
    return;
  }
  const float t = stabilizerStep / (float)RATE_MAIN_LOOP;
  const float dt = 1.0f / CONTROL_HZ;

  if (setpoint->mode.z == modeDisable) {
    // Not flying: motors off, start fresh next time (the fault message is kept)
    const int motor = state.faultMotor;
    const float since = state.faultTime;
    indiWlsReset(&state, &params);
    indiWlsNotifyFault(&state, motor, since);
    for (int i = 0; i < 4; i++) {
      control->normalizedForces[i] = 0.0f;
      forces[i] = 0.0f;
    }
    return;
  }

  const int told = (faultMotor >= 1 && faultMotor <= 4) ? faultMotor - 1 : -1;
  if (told != state.faultMotor) {
    indiWlsNotifyFault(&state, told, t);
  }

  IndiWlsSensors sens;
  IndiWlsReference ref;
  quatToRot(&st->attitudeQuaternion, sens.rot);
  sens.pos[0] = st->position.x;  sens.pos[1] = st->position.y;  sens.pos[2] = st->position.z;
  sens.vel[0] = st->velocity.x;  sens.vel[1] = st->velocity.y;  sens.vel[2] = st->velocity.z;
  sens.gyro[0] = sensors->gyro.x * DEG_TO_RAD;
  sens.gyro[1] = sensors->gyro.y * DEG_TO_RAD;
  sens.gyro[2] = sensors->gyro.z * DEG_TO_RAD;
  // Only absolute position setpoints are tracked; otherwise hold the current position.
  const bool absXY = setpoint->mode.x == modeAbs && setpoint->mode.y == modeAbs;
  const bool absZ = setpoint->mode.z == modeAbs;
  ref.pos[0] = absXY ? setpoint->position.x : sens.pos[0];
  ref.pos[1] = absXY ? setpoint->position.y : sens.pos[1];
  ref.pos[2] = absZ ? setpoint->position.z : sens.pos[2];
  ref.vel[0] = setpoint->velocity.x;  ref.vel[1] = setpoint->velocity.y;  ref.vel[2] = setpoint->velocity.z;
  ref.acc[0] = setpoint->acceleration.x;  ref.acc[1] = setpoint->acceleration.y;  ref.acc[2] = setpoint->acceleration.z;
  ref.yaw = setpoint->attitude.yaw * DEG_TO_RAD;

  indiWlsUpdate(&state, &params, dt, t, &sens, &ref, forces);
  for (int i = 0; i < 4; i++) {
    control->normalizedForces[i] = fminf(fmaxf(forces[i] / params.fMax, 0.0f), 1.0f);
  }
}

/**
 * Fault-tolerant INDI + WLS controller (firmware/indi_wls.c)
 */
PARAM_GROUP_START(indiwls)
/**
 * @brief Value of stabilizer.controller that selects this controller
 */
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, controllerId, &controllerId)
/**
 * @brief Faulty motor the controller has been told about: none(0), M1(1) .. M4(4)
 */
PARAM_ADD(PARAM_UINT8, faultMotor, &faultMotor)
PARAM_ADD(PARAM_FLOAT, mass, &params.mass)
PARAM_ADD(PARAM_FLOAT, fMax, &params.fMax)
PARAM_ADD(PARAM_FLOAT, armD, &params.armD)
PARAM_ADD(PARAM_FLOAT, kappa, &params.kappa)
PARAM_ADD(PARAM_FLOAT, ixx, &params.inertia[0])
PARAM_ADD(PARAM_FLOAT, iyy, &params.inertia[1])
PARAM_ADD(PARAM_FLOAT, izz, &params.inertia[2])
PARAM_ADD(PARAM_FLOAT, motorTau, &params.motorTau)
PARAM_ADD(PARAM_FLOAT, kpPos, &params.kpPos)
PARAM_ADD(PARAM_FLOAT, kdPos, &params.kdPos)
PARAM_ADD(PARAM_FLOAT, kAtt, &params.kAtt)
PARAM_ADD(PARAM_FLOAT, kRate, &params.kRate)
PARAM_ADD(PARAM_FLOAT, faultCap, &params.faultCap)
PARAM_GROUP_STOP(indiwls)

/**
 * Commanded motor thrusts [N]
 */
LOG_GROUP_START(indiwls)
LOG_ADD(LOG_FLOAT, f1, &forces[0])
LOG_ADD(LOG_FLOAT, f2, &forces[1])
LOG_ADD(LOG_FLOAT, f3, &forces[2])
LOG_ADD(LOG_FLOAT, f4, &forces[3])
LOG_GROUP_STOP(indiwls)
