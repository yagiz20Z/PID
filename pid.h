/*
 * pid.h
 *
 *  Created on: Aug 23, 2025
 *      Author: Yagiz
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include <math.h>

#define AKU_ERROR_THRESHOLD                  1.0f

#define AKU_INTEGRAL_MAX_WINDUP_THRESHOLD   20.0f
#define AKU_INTEGRAL_MIN_WINDUP_THRESHOLD  -20.0f

#define MAX_PID                             30.0f
#define MIN_PID                            -30.0f

extern uint8_t Motor_Flag_Status;
// deneme
typedef struct PIDParameters_t{
    float kp;
    float ki;
    float kd;
    float integralBirikimi;
    float prev_error;
    float SRV_RVS;
    float Servo_Calib;
}PIDParameters_t;

extern PIDParameters_t pid;

void PID_Init(PIDParameters_t* pid, float kp, float ki, float kd);   

void PID_reset(PIDParameters_t* pid);    // Hata birikiminin önüne geçeriz

float PID_Currently(PIDParameters_t* pid, float angle, float limit ,uint32_t Dt, uint8_t MotorFlag);



#endif /* INC_PID_H_ */
