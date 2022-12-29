# S-curve-7profile
This library is base on STM32 for control stepper motors with smooth movement.
# Config
To use this library, some configuration must be pre_config

1. 1MHz timer with interupt for each motor

Then in your main fucntion
2. Declare S_curve_t handler for each stepper
	
	eg., S_curve_t Stepper1;
3. Set timer for each stepper
	
	eg., Accel_Stepper_SetTimer(&Stepper1, &htim7);
4. Set pin
	
	Accel_Stepper_SetPin(&Stepper1, step_1_GPIO_Port, step_1_Pin, dir_1_GPIO_Port, dir_1_Pin);
5. To run motor
	
	if(Stepper1.run_status != 1){
		Accel_Stepper_Move(&Stepper1, set_theta1, accel1, accel1, theta1dot);
6. In timer interrupt callback
	
	Accel_Stepper_TIMIT_Handler(&Stepper1);

