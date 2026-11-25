/* PC test for ftc_core.c (no drone needed).
 *   gcc -std=c99 -O2 -I../app_ftc/src test_ftc_core.c ../app_ftc/src/ftc_core.c -lm -o test_ftc_core && ./test_ftc_core
 */
#include <math.h>
#include <stdio.h>

#include "ftc_core.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } else { printf("ok:   %s\n", msg); } } while (0)

int main(void) {
  const ftcPlant_t p = {.mass = 0.029f, .arm = 0.046f, .fMax = 0.12f, .kTorque = 0.0069929f,
                        .J = {1.66e-5f, 1.66e-5f, 2.93e-5f}, .tauMotor = 0.03f};
  const float W = p.mass * 9.81f;
  float f[4];
  bool yawDropped;

  /* healthy hover: four equal forces, yaw kept */
  const float vHover[4] = {W, 0, 0, 0};
  const float upHealthy[4] = {p.fMax, p.fMax, p.fMax, p.fMax};
  ftcMix(&p, vHover, upHealthy, f, &yawDropped);
  CHECK(fabsf(f[0] - W / 4) < 1e-6f && fabsf(f[2] - W / 4) < 1e-6f, "healthy hover splits thrust evenly");
  CHECK(!yawDropped, "healthy hover keeps yaw");

  /* motor 1 capped at 25% thrust: drop yaw, partner M3 matches M1, limits respected */
  const float upFault[4] = {0.25f * p.fMax, p.fMax, p.fMax, p.fMax};
  ftcMix(&p, vHover, upFault, f, &yawDropped);
  printf("      fault forces: %.4f %.4f %.4f %.4f N\n", f[0], f[1], f[2], f[3]);
  CHECK(yawDropped, "fault: yaw is dropped");
  CHECK(f[0] <= 0.25f * p.fMax + 1e-6f, "fault: weak motor within its cap");
  CHECK(fabsf(f[0] - f[2]) < 1e-4f, "fault: diagonal partner matches the weak motor");
  CHECK(fabsf(f[1] - f[3]) < 1e-4f, "fault: the other pair shares the load equally");
  const float a = p.arm * 0.70710678f;
  const float tx = a * (-f[0] - f[1] + f[2] + f[3]);
  const float ty = a * (-f[0] + f[1] + f[2] - f[3]);
  CHECK(fabsf(tx) < 1e-6f && fabsf(ty) < 1e-6f, "fault: no roll/pitch torque at hover");

  /* overloaded after the fault (31 g drone): give every newton the motors can, 2 * 0.25 fMax + 2 fMax */
  const float vHeavy[4] = {0.031f * 9.81f, 0, 0, 0};
  ftcMix(&p, vHeavy, upFault, f, &yawDropped);
  CHECK(fabsf(f[0] + f[1] + f[2] + f[3] - 2.5f * p.fMax) < 1e-5f, "fault overload: all available thrust is used");

  /* closed-loop step at hover gives about weight/4 per motor */
  ftcState_t s;
  const ftcGains_t g = {.kpXY = 4, .kdXY = 3, .kpZ = 6, .kdZ = 4, .tiltMax = 0.436f, .kAtt = 8, .kYaw = 2,
                        .kRate = 25, .lpfHz = 30, .dt = 0.002f, .leadTime = 0.035f};
  ftcInit(&s, &p, &g);
  ftcMeas_t m = {.pos = {0, 0, 0.6f}, .vel = {0, 0, 0}, .R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, .gyro = {0, 0, 0}};
  const ftcSetpoint_t sp = {.pos = {0, 0, 0.6f}, .vel = {0, 0, 0}, .acc = {0, 0, 0}, .yaw = 0};
  for (int i = 0; i < 10; i++) ftcStep(&s, &m, &sp, f);
  CHECK(fabsf(f[0] + f[1] + f[2] + f[3] - W) < 1e-4f, "controller step: total thrust equals weight at hover");

  printf(failures ? "\n%d test(s) FAILED\n" : "\nALL TESTS PASSED\n", failures);
  return failures ? 1 : 0;
}
