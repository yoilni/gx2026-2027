#include "M2006.h"
#include "M2006_PID.h"
#include "M2006_Speed.h"
#include "robot_config.h"
#include <stdint.h>
#include <math.h>

#define M2006_MOTOR_COUNT 4U
#define M2006_ALL_TX_MAILBOXES (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)

// 速度环控制器实例
static SpeedLoopController speed_loop = {0};
static uint8_t feedback_was_fresh[M2006_MOTOR_COUNT];
static float ramped_target_speed[M2006_MOTOR_COUNT];

static uint32_t SpeedLoop_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void SpeedLoop_ExitCritical(uint32_t primask)
{
    if (primask == 0U) __enable_irq();
}

static float SpeedLoop_NormalizeCurrentLimit(int16_t current_limit)
{
    if (current_limit <= 0) return 0.0f;
    if (current_limit > M2006_MAX_CURRENT_LIMIT) return (float)M2006_MAX_CURRENT_LIMIT;
    return (float)current_limit;
}

static void SpeedLoop_ResetPidState(PID_TypeDef *pid)
{
    pid->measure = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;
}

static float SpeedLoop_SlewTarget(float current, float requested,
                                  uint32_t elapsed_ms)
{
    float step;
    float goal = requested;
    float rate = ROBOT_WHEEL_ACCEL_RPM_PER_S;

    /* A commanded reversal must pass through zero instead of flipping the
       wheel-speed target in a single controller update. */
    if ((current > 0.0f && requested < 0.0f) ||
        (current < 0.0f && requested > 0.0f)) {
        goal = 0.0f;
        rate = ROBOT_WHEEL_DECEL_RPM_PER_S;
    } else if (fabsf(requested) < fabsf(current)) {
        rate = ROBOT_WHEEL_DECEL_RPM_PER_S;
    }

    step = rate * 36.0f * (float)elapsed_ms / 1000.0f;
    if (goal > current + step) return current + step;
    if (goal < current - step) return current - step;
    return goal;
}




/**
 * @brief  电机PID初始化
 */
void MotorPID_Init(uint8_t Motor_Number)
{
	
		/*< 初始化PID参数 >*/
  for(int i=0; i<Motor_Number; i++)
  {	

    pid_init(&motor_pid[i]);
		uint16_t maxOutput = 9000;  //最大电流输出限制
		uint16_t integralLimit = 150;  //积分上限阈值
		float deadband = 10;       //死区及误差范围
		uint16_t controlPeriod =0;  // 控制周期及PID计算周期
		int16_t max_err = 8000;     //最大误差
		int16_t  target = 0;     //初始角度
		float kp = 1.5;
		float ki = 0.1;
		float kd = 0;
		
		
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,
															maxOutput,
															integralLimit,
															deadband,
															controlPeriod,
															max_err,
															target,
															kp,
															ki,
															kd);
    
  }
	
	
}





// 初始化速度环
void SpeedLoop_Init(void)
{
    // 初始化PID控制器
    MotorPID_Init(4);
    
    // 初始化速度环参数
	
	    for(uint8_t i = 0U; i < M2006_MOTOR_COUNT; i++) {
        speed_loop.target_speed[i] = 0;
        ramped_target_speed[i] = 0.0f;
        feedback_was_fresh[i] = 0U;
    }
			
    speed_loop.last_update = GetTick();
    
    // 初始化电机速度数组
    for(uint8_t i = 0U; i < M2006_MOTOR_COUNT; i++) {
        speed_loop.current_speed[i] = 0;
    }
}


// 设置左右两侧目标速度；1/4 为左侧，2/3 为右侧。
void SpeedLoop_SetTarget(float speed_left, float speed_right, int16_t current_limit)
{
    uint32_t primask = SpeedLoop_EnterCritical();
    float normalized_limit = SpeedLoop_NormalizeCurrentLimit(current_limit);
    float left_target = speed_left * 36.0f;
    float right_target = speed_right * 36.0f;

    for (uint8_t i = 0U; i < M2006_MOTOR_COUNT; i++) {
        float target = (i == M2006_LEFT_FRONT_INDEX ||
                        i == M2006_LEFT_REAR_INDEX) ? left_target : right_target;
        if (speed_loop.target_speed[i] != target ||
            motor_pid[i].MaxOutput != normalized_limit) {
            SpeedLoop_ResetPidState(&motor_pid[i]);
        }
        speed_loop.target_speed[i] = target;
        motor_pid[i].MaxOutput = normalized_limit;
    }

    SpeedLoop_ExitCritical(primask);
}

/* motor_id is the configured CAN ID (1..4). */
void SpeedLoop_SetMotorTarget(uint8_t motor_id, float speed, int16_t c_limit)
{
    uint32_t primask;

    if (motor_id < 1U || motor_id > 4U) {
        return;
    }

    uint8_t index = motor_id - 1U;
    primask = SpeedLoop_EnterCritical();
    float target = speed * 36.0f;
    float normalized_limit = SpeedLoop_NormalizeCurrentLimit(c_limit);
    if (speed_loop.target_speed[index] != target ||
        motor_pid[index].MaxOutput != normalized_limit) {
        SpeedLoop_ResetPidState(&motor_pid[index]);
    }
    speed_loop.target_speed[index] = target;
    motor_pid[index].MaxOutput = normalized_limit;
    SpeedLoop_ExitCritical(primask);
}

void brake(void)
{
	SpeedLoop_SetTarget(0.0f, 0.0f, stop_current_limit);
}

void SpeedLoop_EmergencyStop(void)
{
    uint32_t primask = SpeedLoop_EnterCritical();
    for (uint8_t i = 0U; i < M2006_MOTOR_COUNT; i++) {
        speed_loop.target_speed[i] = 0.0f;
        ramped_target_speed[i] = 0.0f;
        motor_pid[i].MaxOutput = (float)stop_current_limit;
        SpeedLoop_ResetPidState(&motor_pid[i]);
    }
    SpeedLoop_ExitCritical(primask);
}
// 更新速度环控制(100Hz)
void SpeedLoop_Update(CAN_HandleTypeDef *hcan)
{
    int16_t current_command[M2006_MOTOR_COUNT] = {0};
    uint32_t now = GetTick();
    uint32_t elapsed_ms = now - speed_loop.last_update;
    uint8_t feedback_just_timed_out = 0U;

    /* Avoid one large jump if the task was delayed; normal cadence is 2 ms. */
    if (elapsed_ms > 20U) elapsed_ms = 20U;

    for (uint8_t i = 0U; i < M2006_MOTOR_COUNT; i++) {
        uint8_t feedback_is_fresh =
            (moto_chassis[i].msg_cnt != 0U) &&
            ((uint32_t)(now - moto_chassis[i].last_rx_tick) <= M2006_FEEDBACK_TIMEOUT_MS);

        if (feedback_is_fresh == 0U) {
            if (feedback_was_fresh[i] != 0U) {
                feedback_just_timed_out = 1U;
            }
            feedback_was_fresh[i] = 0U;
            speed_loop.current_speed[i] = 0.0f;
            ramped_target_speed[i] = 0.0f;
            SpeedLoop_ResetPidState(&motor_pid[i]);
            continue;
        }

        feedback_was_fresh[i] = 1U;
        speed_loop.current_speed[i] = moto_chassis[i].speed_rpm;
        ramped_target_speed[i] = SpeedLoop_SlewTarget(
            ramped_target_speed[i], speed_loop.target_speed[i], elapsed_ms);
        motor_pid[i].target = ramped_target_speed[i];
        if (fabsf(ramped_target_speed[i]) < 0.5f &&
            fabsf(speed_loop.current_speed[i]) < motor_pid[i].DeadBand) {
            SpeedLoop_ResetPidState(&motor_pid[i]);
            continue;
        }
        motor_pid[i].f_cal_pid(&motor_pid[i], speed_loop.current_speed[i]);
        if (fabsf(motor_pid[i].output) < 200.0f) motor_pid[i].output = 0.0f;
        current_command[i] = (int16_t)motor_pid[i].output;
    }

    speed_loop.last_update = now;

    if (feedback_just_timed_out != 0U) {
        (void)HAL_CAN_AbortTxRequest(hcan, M2006_ALL_TX_MAILBOXES);
    }

    if (set_moto_current(hcan, current_command[0], current_command[1],
                         current_command[2], current_command[3]) == HAL_BUSY) {
        /* Drop queued stale control frames; the latest command is retried in 2 ms. */
        (void)HAL_CAN_AbortTxRequest(hcan, M2006_ALL_TX_MAILBOXES);
    }
}
