#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

typedef struct
{
  int16_t left_rpm;
  int16_t right_rpm;
} ChassisWheelRpm;

/* Robot-frame wheel RPM, positive forward on both sides. The existing
   periodic speed loop remains the only producer of CAN current commands. */
void ChassisMotion_SetWheelRpm(int16_t left_rpm, int16_t right_rpm, int16_t current_limit);
void ChassisMotion_Stop(void);
void ChassisMotion_EmergencyStop(void);

/* Keep curvature by scaling both targets together. Nonpositive limits stop. */
static inline ChassisWheelRpm ChassisMotion_MixRpm(int16_t forward_rpm, int16_t turn_rpm,
                                                    int16_t wheel_max_rpm)
{
  if (wheel_max_rpm <= 0) return (ChassisWheelRpm){0};
  int32_t left = (int32_t)forward_rpm + turn_rpm;
  int32_t right = (int32_t)forward_rpm - turn_rpm;
  const int32_t left_abs = left < 0 ? -left : left;
  const int32_t right_abs = right < 0 ? -right : right;
  const int32_t magnitude = left_abs > right_abs ? left_abs : right_abs;
  if (magnitude > wheel_max_rpm)
  {
    left = (left * wheel_max_rpm) / magnitude;
    right = (right * wheel_max_rpm) / magnitude;
  }
  return (ChassisWheelRpm){ .left_rpm = (int16_t)left, .right_rpm = (int16_t)right };
}

#endif
