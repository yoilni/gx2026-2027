#include "chassis_motion.h"
#include "M2006_Speed.h"
#include "robot_config.h"

void ChassisMotion_SetWheelRpm(int16_t left_rpm, int16_t right_rpm, int16_t current_limit)
{
  SpeedLoop_SetMotorTarget(ROBOT_LEFT_WHEEL_MOTOR_ID,
      (float)left_rpm * ROBOT_LEFT_WHEEL_FORWARD_SIGN, current_limit);
  SpeedLoop_SetMotorTarget(ROBOT_RIGHT_WHEEL_MOTOR_ID,
      (float)right_rpm * ROBOT_RIGHT_WHEEL_FORWARD_SIGN, current_limit);
}

void ChassisMotion_Stop(void)
{
  brake();
}

void ChassisMotion_EmergencyStop(void)
{
  SpeedLoop_EmergencyStop();
}
