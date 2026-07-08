/*
 * pid.c
 *
 *  Created on: Aug 23, 2025
 *      Author: Yagiz
 */

#include "pid.h"
#include <string.h> 

PIDParameters_t pid;


static saturation(float min, float max, float value)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

/**
 * @brief 
 * 
 * @param pid 
 * @param kp 
 * @param ki 
 * @param kd
 * 
 * @note nurada kp, ki ve kd değerlerin ataması yapılmaktadır
 *  
 */
void PID_Init(PIDParameters_t* pid, float kp, float ki, float kd){

    if (pid == NULL) {
        return; 
    }
    memset(pid, 0, sizeof(PIDParameters_t));

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

}

/**
 * @brief 
 * 
 * @param pid 
 * 
 * @note İntegral birikiminin önüne geçer
 */
void PID_reset(PIDParameters_t* pid){        
    if (pid == NULL){
        return;
    }
    pid->integralBirikimi = 0.0f;
    pid->prev_error = 0.0f;

}   

/**
 * @brief PID cikisini hesaplar.
 *
 * @param pid       PID yapisinin adresi
 * @param angle     Olculen anlik deger
 * @param limit     Hedef deger
 * @param Dt_ms     Ornekleme suresi, milisaniye cinsinden
 * @param MotorFlag 1 ise PID aktif, 0 ise PID resetlenir
 *
 * @return MIN_PID ile MAX_PID arasinda sinirlanmis PID cikisi
 */
float PID_Currently(PIDParameters_t* pid,
                    float angle,
                    float limit,
                    uint32_t Dt_ms,
                    uint8_t MotorFlag)
{
    if (pid == NULL) {
        return 0.0f;
    }

    /* Sifira bolme ve hatali zaman olcumunu engeller */
    if (Dt_ms == 0U) {
        return 0.0f;
    }

    /* Motor pasifse PID hafizasini temizle ve cikisi sifirla */
    if (MotorFlag == 0U) {
        PID_reset(pid);
        return 0.0f;
    }

    /* PID hesabinda zaman saniye cinsinden kullanilir */
    float dt = (float)Dt_ms / 1000.0f;

    float error = limit - angle;

    /* P: Oransal kontrol */
    float proportional = pid->kp * error;

    /* I: Integral kontrol */
    if (fabsf(error) < AKU_ERROR_THRESHOLD) {
        pid->integralBirikimi = 0.0f;
    } else {
        pid->integralBirikimi += error * dt;

        pid->integralBirikimi = saturation(
            AKU_INTEGRAL_MIN_WINDUP_THRESHOLD,
            AKU_INTEGRAL_MAX_WINDUP_THRESHOLD,
            pid->integralBirikimi
        );
    }

    float integral = pid->ki * pid->integralBirikimi;

    /* D: Turev kontrol */
    float derivative_Coefficient = (error - pid->prev_error) / dt;
    float derivative = pid->kd * derivative_Coefficient;

    pid->prev_error = error;

    float PID_output = proportional + integral + derivative;

    return saturation(MIN_PID, MAX_PID, PID_output);
}


