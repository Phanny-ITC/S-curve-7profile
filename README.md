# S-curve-7profile
This library is base on STM32 for control stepper motors with smooth movement.
For linear acceleration can be found at https://github.com/Phanny-ITC/STM32/tree/main/Accel_Stepper_STM32
# Config
To use this library, some configurations must be pre_config

1. 1MHz timer with interupt for each motor

Then in your main fucntion

2. Declaire S_curve_t handler for each stepper
	```
	 S_curve_t Stepper1;
	```
3. Set timer for each steppers
	```
	Accel_Stepper_SetTimer(&Stepper1, &htim7);
	```
4. Set pin
	```
	Accel_Stepper_SetPin(&Stepper1, step_1_GPIO_Port, step_1_Pin, dir_1_GPIO_Port, dir_1_Pin);
	```
5. To run motor
	```
	if(Stepper1.run_status != 1){
		Accel_Stepper_Move(&Stepper1, step_num, accel1, accel1, speed);
	}
	```
6. In timer interrupt callback
	```
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
		if(htim->Instance == TIM7){//stepper 1
			Accel_Stepper_TIMIT_Handler(&Stepper1);
		}
	}
	```
