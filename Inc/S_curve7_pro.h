#ifndef S_CURVE7_PRO_H
#define S_CURVE7_PRO_H

#include "stdlib.h"
#include "math.h"
#include <stdint.h>
#include "gpio.h"
#include "tim.h"

#define SPR 6400                //number of step per round 1/32
#define T_FREQ 2000000          //timer frequency Hz
#define ALPHA (6283185/SPR)   // 2*pi/spr * 1000000
#define MIN_FREQ 1000          // 1kHz
// #define MAX_ARR T_FREQ/MIN_FREQ  (max_arr = f*sqrt(2*alpha/a_max);)
#define MAX_ARR 2000

typedef enum{
	STOP,
	START
}ramp_state_t;
typedef enum{
	STEPPER1,
	STEPPER2,
//	STEPPER3,
//	STEPPER4,
//	STEPPER5,
//	STEPPER6
}Stepper_t;
//typedef void (*execute_phase_t)(Stepper_t stepper);
typedef struct{

    uint32_t v_accel;              	/*Acceleration speed <rad/s>: max speed reached during the movement*/
    uint32_t theta_accel;          	/*Acceleration angle: the stroke at the end of acceleration correspond to t_accel <rad>; (for distance "s_accel" <m>)*/
    uint32_t theta_v;              	/*Acceleration angle: the stroke at the end of the movement correspond to t_v <rad>; (for distance "s_v" <m>)*/
    uint32_t theta;                	/*Angle to move <rad>; (for distance "s" <m>)*/
    uint32_t accel_max;            	/*Max acceleration */
    uint32_t v_max;                	/*Max speed*/
    uint32_t J;                    	/*constant jerk*/
    uint32_t arr;                  	/*timer autoreload register (counter period)*/
    uint32_t v1,v2,v3,v4,v5,v6;    	/*speed at the end of each phase*/
    uint32_t accel;                	/*Acceleration coefficient*/
    uint32_t v;                    	/*Output speed of each phase*/
    uint32_t step_count,step_change,step1,step2,step3,step4,step5,step6;
    uint16_t min_arr;
    float tj;                  	   	/*time constant for jerk*/
    float ta;                   	/*time constant for acceleration*/
    float tv;                  		/*time constant for reach to the end of movement(v_max)*/
    float t,t_change;                   		/*time*/
    float t1,t2,t3,t4,t5,t6,t7;		/*time of each phase*/
    signed int a1,a2,a3,a5,a6;		/*acceleration at the end of each phase*/
    signed int step;            	/*Number of step to move*/
//    uint16_t max_arr;           	/*Maximum autoreload*/
	uint16_t Step_Pin;          	/*GPIO PIN for step pin*/
	uint16_t Dir_Pin;           	/*GPIO PIN for direction pin*/
    GPIO_TypeDef* Step_Port;    	/*GPIO port for step pin*/
	GPIO_TypeDef* Dir_Port;     	/*GPIO port for direction pin*/
	TIM_HandleTypeDef* htim;    	/*Timer structure*/
    ramp_state_t run_state;
}S_curve_t;

void Accel_Stepper_Move(S_curve_t* stepper, signed int step, uint8_t accel, float jerk, uint16_t rpm);
void Accel_Stepper_SetTimer(S_curve_t* stepper, TIM_HandleTypeDef* htim);
void Accel_Stepper_SetPin(S_curve_t* stepper, GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin);
void Accel_Stepper_TIMIT_Handler(S_curve_t* stepper);
void run_stepper(S_curve_t* stepper);
void execute_phase_1(S_curve_t* stepper);
void execute_phase_2(S_curve_t* stepper);
void execute_phase_3(S_curve_t* stepper);
void execute_phase_4(S_curve_t* stepper);
void execute_phase_5(S_curve_t* stepper);
void execute_phase_6(S_curve_t* stepper);
void execute_phase_7(S_curve_t* stepper);
#endif // S_CURVE7_PRO_H
