
#include "S_curve7_pro.h"
extern uint32_t cnt;
//S_curve_t sc_accel[sizeof(Stepper_t)];
/*
 * Accel_Stepper_Move
 * stepper : Number of which stepper use found @ Stepper_t
 * step : Number of step to run
 * accel : acceleration <rad/s^2>
 * jerk : constant jerk limit <rad/s^3>
 * rpm : speed at run state
 */
void Accel_Stepper_Move(S_curve_t* stepper, signed int step, uint8_t accel, float jerk, uint16_t rpm){

    if(step < 0){
        HAL_GPIO_WritePin(stepper->Dir_Port, stepper->Dir_Pin, 0);
        stepper->step = -step;
    }
    else{
    	stepper->step = step;
        HAL_GPIO_WritePin(stepper->Dir_Port, stepper->Dir_Pin, 1);
    }
    stepper->accel_max = 1000000*accel;
//    stepper->max_arr = T_FREQ * sqrtf(2*ALPHA/stepper->accel_max);
    stepper->v_max = 104720*rpm; //2*3.141592*rpm/60*1000000;
    stepper->J = 1000000*jerk;
    stepper->theta = ALPHA*stepper->step;//theta


    stepper->v_accel = stepper->accel_max/stepper->J *stepper->accel_max;

    stepper->theta_accel = 2*(stepper->accel_max/stepper->J * stepper->accel_max
    								/stepper->J * stepper->accel_max);
    if (stepper->v_max < stepper->v_accel)
        stepper->theta_v = 2*stepper->v_max*sqrtf(stepper->v_max/stepper->J);
    else
        stepper->theta_v = stepper->v_max*((stepper->v_max/stepper->accel_max)
        							+(stepper->accel_max/stepper->J));

    if (((stepper->v_max>stepper->v_accel) && (stepper->theta<stepper->theta_accel))
    		|| ((stepper->v_max<stepper->v_accel) && ((stepper->theta<stepper->theta_accel)
    			&& (stepper->theta<stepper->theta_v)))){//traj B&C.2

        stepper->tj =(float) powf((float)stepper->theta/(2*stepper->J), 1.0/3);
        stepper->ta = stepper->tj;

        stepper->tv = 2*stepper->tj;
        }

    else if (((stepper->v_max<stepper->v_accel) && (stepper->theta>stepper->theta_accel))
    		|| ((stepper->v_max<stepper->v_accel) && ((stepper->theta<stepper->theta_accel)
        		&& (stepper->theta>stepper->theta_v)))){//traj A&C.1
        stepper->tj =(float) sqrtf(stepper->v_max/stepper->J);
        stepper->ta = stepper->tj;
        stepper->tv = (float) stepper->theta/stepper->v_max;
    }

    else if (((stepper->v_max > stepper->v_accel) && (stepper->theta > stepper->theta_accel))
    		&&(stepper->theta > stepper->theta_v)){//traj D.1
        stepper->tj = (float) accel/jerk;
        stepper->ta = (float) stepper->v_max/stepper->accel_max;
        stepper->tv = (float) stepper->theta/stepper->v_max;
    }

    else if (((stepper->v_max > stepper->v_accel) && (stepper->theta > stepper->theta_accel))
    		&&(stepper->theta < stepper->theta_v)){//traj D.2
        stepper->tj = (float) stepper->accel_max/stepper->J;
        float a3 =  accel*accel*accel;
        float thetaJ2 = (float) 4*stepper->theta/1000000 * jerk*jerk;
        float aJ2 = accel*jerk*jerk;

        stepper->ta = (float) 1/2 * (sqrtf((thetaJ2+ a3)/aJ2)-(float) (stepper->accel_max/stepper->J));
        stepper->tv = stepper->ta + stepper->tj;
    }
    /*Calculate time sequence*/

    stepper->t1 = stepper->tj;
    stepper->t2 = stepper->ta;
    stepper->t3 = stepper->tj + stepper->ta;
    stepper->t4 = stepper->tv;
    stepper->t5 = stepper->tj + stepper->tv;
    stepper->t6 = stepper->tv + stepper->ta;
    stepper->t7 = stepper->tv + stepper->ta + stepper->tj;
//    uint32_t stp7 = (float) stepper->step/(stepper->t7*stepper->t7);
//    stepper->step1 = stepper->t1*stp7;
//    stepper->step2 = stepper->t2*stp7;
//    stepper->step3 = stepper->t3*stp7;
//    stepper->step4 = stepper->t4*stp7;
//    stepper->step5 = stepper->t5*stp7;
//    stepper->step6 = stepper->t6*stp7;
    /*Start acceleration*/
//    __HAL_TIM_SET_AUTORELOAD(stepper->htim,stepper->max_arr);
    __HAL_TIM_SET_AUTORELOAD(stepper->htim, MAX_ARR);
    stepper->min_arr = ALPHA/stepper->v_max *T_FREQ;

    stepper->t = 0;
    stepper->step_count = 0;
    stepper->arr = 1000;
//    stepper->v = ALPHA*T_FREQ/stepper->arr;

    HAL_TIM_Base_Start_IT(stepper->htim);
    stepper->run_state = START;
}
/*in case t <= t1 : execute_phase_1*/
void execute_phase_1(S_curve_t* stepper){
        //if(s_c->t <= s_c->t1)

//    uint32_t j = ;
//    float T2 = stepper->t * stepper->t;
	stepper->a1 = stepper->J * stepper->t;
    stepper->v  = stepper->a1/2 * stepper->t + 981747;
    stepper->v1 = stepper->v;
    stepper->a2 = stepper->a1;

}

/*in case t > t1 & t <= t2 : execute_phase_2*/
void execute_phase_2(S_curve_t* stepper){
        // int j = 0;

	float dt = stepper->t - stepper->t1;
//	stepper->a2 = stepper->a1;
	stepper->v  = stepper->v1 + stepper->a2*dt;
        // p(i+1) = p1 + v1*(t-t1)+1/2 * stepper->accel*(t-t1)^2;
//    if(stepper->v > stepper->v_max){
//    	stepper->v = stepper->v_max;
//    }
    stepper->v2 = stepper->v;

}
/*in case t > t2 & t <= t3 : execute_phase_3*/
void execute_phase_3(S_curve_t* stepper){

    if((uint16_t) (stepper->t1*100)  == (uint16_t) (stepper->t2*100)){
    	stepper->v2 = stepper->v1;
//        stepper->a2 = stepper->a1;
    }
//    uint32_t j = -stepper->J;
    float dt = (stepper->t - stepper->t2);
//    stepper->a3 = stepper->a2 - j*dt;
//    uint32_t Jt2 = stepper->J/2 *dt*dt;
    stepper->v = stepper->v2 + (stepper->a2 - stepper->J/2 *dt)*dt;
        // p(i+1) = p2 + v2*(t-t2)+1/2* a2*(t-t2)^2+1/6 * j*(t-t2)^3;
//    if(stepper->v > stepper->v_max){
//    	stepper->v = stepper->v_max;
//    }
    stepper->v3 = stepper->v;
        // p3 = p(i+1);


}
/*in case t > t3 & t <= t4 : execute_phase_4*/
void execute_phase_4(S_curve_t* stepper){

        // int j = 0;
        // stepper->a4 = 0;

	stepper->v = stepper->v3;
        // p(i+1) = p3+v_max*(t-t3);
//    stepper->v4 = stepper->v;
        // p4 = p(i+1);


}
/*in case t > t4 & t <= t5 : execute_phase_5*/
void execute_phase_5(S_curve_t* stepper){
//    if(stepper->t3 == stepper->t4){
//        stepper->v4 = stepper->v3;
//    }

//    signed int j = -stepper->J;
    float dt = stepper->t - stepper->t4;
    stepper->a5 = stepper->J*dt*(-1);
    signed int Jt2 = stepper->a5/2 *dt;
    stepper->v = stepper->v3 + Jt2;
        // p(i+1) = p4+v4*(t-t4) + 1/6 *j*(t-t4)^3;
//    if(stepper->v > stepper->v_max){
//    	stepper->v = stepper->v_max;
//    }
    stepper->v5 = stepper->v;
        // p5 = p(i+1);

}
/*in case t > t5 & t <= t6 : execute_phase_6*/
void execute_phase_6(S_curve_t* stepper){
        // int j = 0;

	float dt = stepper->t - stepper->t5;
	stepper->a6 = stepper->a5;
    stepper->v = stepper->v5 + stepper->a5*dt;
        // p(i+1)= p5 + v5*(t-t5) + 1/2 *a5*(t-t5)^2;
//    if(stepper->v>stepper->v_max){
//    	stepper->v = stepper->v_max;
//    }
    stepper->v6 = stepper->v;
        // p6 = p(i+1);

}
/*in case t > t6 & t <= t7 : execute_phase_7*/
void execute_phase_7(S_curve_t* stepper){
    if((uint16_t) (stepper->t5*100) == (uint16_t) (stepper->t6*100)){
    	stepper->v6 = stepper->v5;
    	stepper->a6 = stepper->a5;
    }    
//    uint32_t j = stepper->J;
    float dt = stepper->t - stepper->t6;
//        stepper->a7 = stepper->a6 + j*(stepper->t - stepper->t6);
//    uint32_t Jt2 = j/2 *dt*dt;
    stepper->v = stepper->v6 + (stepper->a6 + stepper->J/2 *dt)*dt;
        // p(i+1)=p6 + v6*(t-t6) + 1/2 *a6*(t-t6)^2 + 1/6 *j*(t-t6)^3;
        // a=a7;
        // p7 = p(i+1);
        // v7 = v(i+1);

}
//execute_phase_t execute_phase[7]={execute_phase_1,execute_phase_2,execute_phase_3,execute_phase_4,execute_phase_5,execute_phase_6,execute_phase_7};
/*
 * Accel_Stepper_TIMIT_Handler
 * stepper : Num of which stepper use found @ Stepper_t
 */
void Accel_Stepper_TIMIT_Handler(S_curve_t* stepper){

	static float arr = 0;
//	static float arr_old = 1000;
	__HAL_TIM_SET_AUTORELOAD(stepper->htim, stepper->arr);

	switch(stepper->run_state) {
		case STOP:
			stepper->step_count = 0;
               
		     // Stop Timer/Counter 1.
		   	HAL_TIM_Base_Stop_IT(stepper->htim);
		   	stepper->t = 0;
//		      status.running = false;
		   	break;
	    case START:

//	    	cnt++;

	        if(stepper->t <= stepper->t1){
                execute_phase_1(stepper);
	        }
	        else if (stepper->t > stepper->t1 && stepper->t <= stepper->t2){
            	execute_phase_2(stepper);
            }
            else if (stepper->t > stepper->t2 && stepper->t <= stepper->t3){
            	execute_phase_3(stepper);
            }
            else if (stepper->t > stepper->t3 && stepper->t <= stepper->t4){
            	execute_phase_4(stepper);
            }
            else if (stepper->t > stepper->t4 && stepper->t <= stepper->t5){
            	execute_phase_5(stepper);
            }
            else if (stepper->t > stepper->t5 && stepper->t <= stepper->t6){
            	execute_phase_6(stepper);
            }
            else if (stepper->t > stepper->t6 && stepper->t <= stepper->t7){
            	execute_phase_7(stepper);
            	if(stepper->v < 1963495) stepper->v = 1963495;
            }

            break;
    }
    arr = ALPHA*T_FREQ /(float) stepper->v ;
//    float vref = ALPHA*T_FREQ /arr;
    if(arr > MAX_ARR){
        arr = MAX_ARR;
    }else if(arr < stepper->min_arr){
    	arr = stepper->min_arr;
    }
    stepper->arr = arr-1;
//    int g = (arr - arr_old);
//    if(g==0) g=1;
//    stepper->t_change = 0.001/g;
//    arr_old = arr;
//    stepper->step_change = (float) stepper->t_change*T_FREQ/arr;
//    (stepper->htim)->Instance->ARR = (uint16_t) stepper->arr;
    stepper->t = stepper->t +  0.001;//sample time (s)
//	stepper->t = stepper->t +  arr/T_FREQ;//sample time (s)



//    cnt++;

}
/*
 * Set GPIO for each stepper
 * stepper : Num of whitch stepper use found @ Stepper_t
 * step_port : GPIO port of step pin
 * step_pin : gpio pin number of step pin
 * dir_port : GPIO port of direction pin
 * dir_pin : gpio pin number of direction pin
 */
void Accel_Stepper_SetPin(S_curve_t* stepper, GPIO_TypeDef* step_port,
		uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin)
{
	stepper->Step_Port = step_port;
	stepper->Dir_Port = dir_port;
	stepper->Step_Pin = step_pin;
	stepper->Dir_Pin = dir_pin;
}
/*
 * Set Timer for each motor
 * stepper : Num of which stepper use found @ Stepper_t
 * timer : pointer to timer typedef(Which timer is use for control speed)
 */
void Accel_Stepper_SetTimer(S_curve_t* stepper, TIM_HandleTypeDef* timer){
	stepper->htim = timer;
}
void run_stepper(S_curve_t* stepper){

	HAL_GPIO_WritePin(stepper->Step_Port, stepper->Step_Pin, 1);
    HAL_GPIO_WritePin(stepper->Step_Port, stepper->Step_Pin, 1);
    HAL_GPIO_WritePin(stepper->Step_Port, stepper->Step_Pin, 0);
	HAL_GPIO_WritePin(stepper->Step_Port, stepper->Step_Pin, 0);
	stepper->step_count++;
//	if(stepper->step_change != 0){
//		if(stepper->step_change<0){//accel
//			stepper->step_change ++;
//			stepper->arr--;
//			__HAL_TIM_SET_AUTORELOAD(stepper->htim, stepper->arr);
//		}
//		else {
//			stepper->step_change --;//decel
//			stepper->arr++;
//			__HAL_TIM_SET_AUTORELOAD(stepper->htim, stepper->arr);
//		}
//
//	}
    if(stepper->step_count >= stepper->step){
    	stepper->run_state = STOP;
        stepper->step_count = 0;

	     // Stop Timer/Counter 1.
	   	HAL_TIM_Base_Stop_IT(stepper->htim);
	   	stepper->t = 0;
    }
}
