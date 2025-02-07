/*
 * motor.c
 *
 * Created: 10.08.2024 15:38:01
 *  Author: H.Dani
 */ 
 

#include "motor.h"

Motor init_motor(uint8_t motortype, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2,
				volatile uint8_t *ddreg, uint8_t ddrpin1, uint8_t ddrpin2, volatile int32_t encoder, 
				volatile uint8_t *ddrpwm, uint8_t pwmpin, volatile uint16_t *ocr, uint8_t max_speed){
	Motor motor;
	motor.encoder = encoder;
	motor.motortype = motortype;
	motor.portdirpin = portdirpin;
	motor.dirpin1 = dirpin1;
	motor.dirpin2 = dirpin2;
	motor.ddreg = ddreg;
	motor.ddrpin1 = ddrpin1;
	motor.ddrpin2 = ddrpin2;
	motor.ddrpwm = ddrpwm;
	motor.pwmpin = pwmpin;
	motor.ocr = ocr;
	motor.max_speed = max_speed;
	motor.pwm_value = max_speed;

	//SET DIR PIN OUTPUTS
	set_output(motor.ddreg, motor.ddrpin1);
	set_output(motor.ddreg, motor.ddrpin2);

	if(motortype == 1){			
		motor.ticks_per_rev = 12; //MICROMOTOR
		//SET PWM PIN OUTPUTS
		set_output(motor.ddrpwm, motor.pwmpin);
	}		
	else if(motortype == 2) motor.ticks_per_rev = 700; //BASE MOTOR

	return motor;
}

void init_pid(Motor *motor, float kp, float kd, float ki){
	motor->pid.Kp = kp;
	motor->pid.Kd = kd;
	motor->pid.Ki = ki;
	motor->pid.prev_error = 0;
	motor->pid.setpoint = 0;
	motor->pid.critical_delta = 200.0;
	motor->pid.e_prop = 0;
	motor->pid.e_dot = 0;
	motor->pid.e_int = 0;
	motor->pid.us_time = 0;
	motor->pid.d_time = 0;
	motor->pid.prev_time = 0;
}

inline void fwd(Motor *motor, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2){
	if(motor->motortype == 1){			//MICROMOTOR
		pin_high(motor->portdirpin, motor->dirpin1);
		pin_low(motor->portdirpin, motor->dirpin2);
	}else if(motor->motortype == 2){	//BASE MOTOR
		motor->dirpin1 = motor->pwm_value;
		motor->dirpin2 = 0;
	}
}

inline void bwd(Motor *motor, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2){
	if(motor->motortype == 1){			//MICROMOTOR
		pin_low(motor->portdirpin, motor->dirpin1);
		pin_high(motor->portdirpin, motor->dirpin2);
	}else if(motor->motortype == 2){	//BASE MOTOR
		motor->dirpin1 = 0;
		motor->dirpin2 = motor->pwm_value;
	}
}

inline void stop(Motor *motor, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2){
	if(motor->motortype == 1){			//MICROMOTOR
		pin_low(motor->portdirpin, motor->dirpin1);
		pin_low(motor->portdirpin, motor->dirpin2);
	}else if(motor->motortype == 2){	//BASE MOTOR
		motor->dirpin1 = 0;
		motor->dirpin2 = 0;
	}
}

inline void calculatePID(Motor *motor){ 
	motor->pid.us_time = micros();
	motor->pid.d_time = motor->pid.us_time - motor->pid.prev_time;
	motor->pid.prev_time = motor->pid.us_time;

	// FIX VARIABLE TYPE ISSUES !!!!!!!!!!

	motor->pid.e_prop	= motor->pid.setpoint - motor->pid.current_pos;
	motor->pid.e_dot	= (motor->pid.e_prop - motor->pid.prev_error)/motor->pid.d_time;
	motor->pid.e_int	= motor->pid.e_int + (motor->pid.e_prop * motor->pid.d_time);

	motor->pid.ctrl_signal = (motor->pid.Kp * motor->pid.e_prop) + 
							 (motor->pid.Kd * motor->pid.e_dot) + 
							 (motor->pid.Ki * motor->pid.e_int);
	
	motor->pid.prev_error = motor->pid.e_prop;
}

void move_abs(Motor *motor, int32_t setpoint, volatile int32_t currentpos){
	motor->pid.current_pos = currentpos;
	motor->pid.setpoint = setpoint;
	
	calculatePID(motor);

	if(motor->pid.ctrl_signal > motor->pid.critical_delta)
		motor->pwm_value = motor->max_speed;
	else
		motor->pwm_value = ((abs(motor->pid.ctrl_signal)/motor->pid.critical_delta)*motor->max_speed);

	if(motor->motortype == 1) *motor->ocr = motor->pwm_value;

	if(motor->pid.ctrl_signal > MIN_POS_DELTA) fwd(motor, motor->portdirpin, motor->dirpin1, motor->dirpin2);
	else if(motor->pid.ctrl_signal < -MIN_POS_DELTA) bwd(motor, motor->portdirpin, motor->dirpin1, motor->dirpin2);
	else stop(motor, motor->portdirpin, motor->dirpin1, motor->dirpin2);
}