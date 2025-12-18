#ifndef MecanumKinematics_h
#define MecanumKinematics_h
#include <math.h>

const float MAX_WHEEL_JUMP = 15.0f;

struct WheelRotations {
  float m1, m2, m3, m4;
};

float normalize_angle(float angle) {
  while (angle > M_PI) angle -= 2.0f * M_PI;
  while (angle <= -M_PI) angle += 2.0f * M_PI;
  return angle;
}

// 1. INVERSE KINEMATICS (Standard X-Forward Logic)
// dx = Forward/Back, dy = Strafe Left/Right
WheelRotations mecanum_ik(float dx, float dy, float dtheta, float r, float L, float W) {
  float l = L + W;
  // Standard Rotation: Left (-) vs Right (+)
  float m1 = (dx - dy - l * dtheta) / r;
  float m2 = (dx + dy + l * dtheta) / r;
  float m3 = (dx + dy - l * dtheta) / r;
  float m4 = (dx - dy + l * dtheta) / r;
  return { m1, m2, m3, m4 };
}

// 2. PATH PLANNING (World -> Robot)
// This converts World X/Y into Robot Forward (dx) / Strafe (dy)
WheelRotations mecanum_to_pose(float x_curr, float y_curr, float theta_curr,
                               float x_goal, float y_goal, float theta_goal,
                               float r, float L, float W) {
  float dx_world = x_goal - x_curr;
  float dy_world = y_goal - y_curr;
  float dtheta = normalize_angle(theta_goal - theta_curr);

  float cos_th = cosf(theta_curr);
  float sin_th = sinf(theta_curr);

  // Rotation Matrix
  // Projects the world vector onto the Robot's X (Forward) and Y (Left) axes
  float dx = cos_th * dx_world + sin_th * dy_world;
  float dy = -sin_th * dx_world + cos_th * dy_world;

  return mecanum_ik(dx, dy, dtheta, r, L, W);
}
void updateOdometry() {
  float d[4];

  // 1. Read & filter
  for (int i = 0; i < 4; i++) {
    float delta = w[i].current - prev_w[i];

    if (fabs(delta) > MAX_WHEEL_JUMP) {
      delta = 0;
      w[i].current = prev_w[i];
    }

    d[i] = delta;
    prev_w[i] = w[i].current;
  }

  float R = WHEEL_RADIUS;
  float LxLy = HALF_LENGTH + HALF_WIDTH;

  // 2. Inverse mecanum kinematics (MATCHES IK)
  // X = forward
  float robot_dx_local = (-d[0] + d[1] + d[2] - d[3]) * (R / 4.0f);

  // Y = strafe
  float robot_dy_local = (d[0] + d[1] + d[2] + d[3]) * (R / 4.0f);

  float robot_dth_local =
    (-d[0] + d[1] - d[2] + d[3]) * (R / (4.0f * LxLy));

  // 3. Transform to world frame
  float cos_th = cosf(current_th);
  float sin_th = sinf(current_th);

  current_x += robot_dx_local * cos_th - robot_dy_local * sin_th;
  current_y += robot_dx_local * sin_th + robot_dy_local * cos_th;
  current_th = normalize_angle(current_th + robot_dth_local);
}

#endif