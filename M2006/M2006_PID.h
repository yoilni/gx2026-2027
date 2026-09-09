#ifndef __M2006_PID_H
#define __M2006_PID_H

#include "main.h"
#include <stdint.h>



typedef enum
{

	PID_Position,
	PID_Speed
	
}PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target;							//目标值
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					//测量值
	float   err;							//误差
	float   last_err;      		//上次误差
	
	float pout;
	float iout;
	float dout;
	
	float output;						//本次输出
	float last_output;			//上次输出
	
	float MaxOutput;				//输出限幅
	float IntegralLimit;		//积分限幅
	float DeadBand;			  //死区（绝对值）
	float ControlPeriod;		//控制周期
	float  Max_Err;					//最大误差
	
					  uint32_t thistime;
					uint32_t lasttime;
						uint8_t dtime;	
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
				   PID_ID id,
				   uint16_t maxOutput,   //最大电流输出限制
				   uint16_t integralLimit,  //积分上限阈值
				   float deadband,       //死区及误差范围
				   uint16_t controlPeriod,  // 控制周期及PID计算周期
					int16_t max_err,      //最大误差
					int16_t  target,     //初始角度
				   float kp,
				   float ki,
				   float kd);
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid计算
}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
uint32_t GetTick(void);


extern PID_TypeDef motor_pid[4];
#endif

