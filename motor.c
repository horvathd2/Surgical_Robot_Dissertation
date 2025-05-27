/*
 * motor.c
 *
 * Created: 10.08.2024 15:38:01
 *  Author: H.Dani
 */ 
 

#include "motor.h"

Motor init_motor(uint8_t motortype, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2,
				volatile uint8_t *ddreg, uint8_t ddrpin1, uint8_t ddrpin2, 
				volatile uint8_t *ddrpwm, uint8_t pwmpin, volatile uint16_t *ocr1, 
				volatile uint16_t *ocr2 , uint8_t max_speed){
	Motor motor;
	motor.motortype		= motortype;
	motor.portdirpin	= portdirpin;
	motor.dirpin1		= dirpin1;
	motor.dirpin2		= dirpin2;
	motor.ddreg			= ddreg;
	motor.ddrpin1		= ddrpin1;
	motor.ddrpin2		= ddrpin2;
	motor.ddrpwm		= ddrpwm;
	motor.pwmpin		= pwmpin;
	motor.ocr1			= ocr1;
	motor.ocr2			= ocr2;
	motor.max_speed		= max_speed;
	motor.pwm_value		= max_speed;
	motor.min_speed		= 50;

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
	motor->pid.Kp				= kp;
	motor->pid.Kd				= kd;
	motor->pid.Ki				= ki;
	motor->pid.prev_error		= 0;
	motor->pid.setpoint			= 0;
	motor->pid.critical_delta	= 2 * motor->ticks_per_rev;
	motor->pid.e_prop			= 0;
	motor->pid.e_dot			= 0;
	motor->pid.e_int			= 0;
	motor->pid.us_time			= 0;
	motor->pid.d_time			= 0;
	motor->pid.prev_time		= 0;
	motor->pid.last_pos			= 0;
	motor->pid.prev_stall_time	= 0;
}

void init_pwm(void){
	//SET THE TIMER COUNTER CONTROL REGISTERS TO FAST PWM MODE WITH CLEAR ON COMPARE MATCH
	//MICRO MOTOR PWM SETUP
	TCCR0A |= (1 << COM0A1) | (1 << WGM01)  | (1 << WGM00);
	TCCR0B |=  (1 << CS00)  | (1 << WGM02);
	
	TCCR1A |= (1 << COM1B1) | (1 << WGM10);
	TCCR1B |=  (1 << CS10)  | (1 << WGM12);

	//BASE MOTOR PWM SETUP
	TCCR2A |= (1 << COM2B1) | (1 << WGM20); //| (1 << WGM21);
	TCCR2B |=  (1 << CS20);

	TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM40);
	TCCR4B |=  (1 << CS40);  //| (1 << WGM42);
}

void fwd(Motor *motor){
	motor->moving_fwd = 1;
	motor->moving_bwd = 0;
	if(motor->motortype == 1){			//MICROMOTOR
		pin_high(motor->portdirpin, motor->dirpin1);
		pin_low(motor->portdirpin, motor->dirpin2);
	}else if(motor->motortype == 2){	//BASE MOTOR
		*motor->ocr1 = abs(motor->pwm_value);
		*motor->ocr2 = 0;
	}
}

void bwd(Motor *motor){
	motor->moving_fwd = 0;
	motor->moving_bwd = 1;
	if(motor->motortype == 1){			//MICROMOTOR
		pin_low(motor->portdirpin, motor->dirpin1);
		pin_high(motor->portdirpin, motor->dirpin2);
	}else if(motor->motortype == 2){	//BASE MOTOR
		*motor->ocr1 = 0;
		*motor->ocr2 = abs(motor->pwm_value);
	}
}

void stop(Motor *motor){
	motor->moving_fwd = 0;
	motor->moving_bwd = 0;
	if(motor->motortype == 1){			//MICROMOTOR
		pin_low(motor->portdirpin, motor->dirpin1);
		pin_low(motor->portdirpin, motor->dirpin2);
	}else if(motor->motortype == 2){	//BASE MOTOR
		*motor->ocr1 = 0;
		*motor->ocr2 = 0;
	}
}

void calculatePID(Motor *motor){ 
	motor->pid.us_time	= micros();
	motor->pid.d_time	= (motor->pid.us_time - motor->pid.prev_time);

	motor->pid.e_prop	= motor->pid.setpoint - motor->pid.current_pos;
	motor->pid.e_dot	= (motor->pid.e_prop - motor->pid.prev_error)/motor->pid.d_time;
	motor->pid.e_int	= motor->pid.e_int + (motor->pid.e_prop * motor->pid.d_time);

	motor->pid.ctrl_signal = (motor->pid.Kp * motor->pid.e_prop) + 
							 (motor->pid.Kd * motor->pid.e_dot) + 
							 (motor->pid.Ki * motor->pid.e_int);
	
	motor->pid.prev_error	= motor->pid.e_prop;
	motor->pid.prev_time	= motor->pid.us_time;

	if (motor->pid.ctrl_signal > motor->max_speed) motor->pid.ctrl_signal = motor->max_speed;
	if (motor->pid.ctrl_signal < -motor->max_speed) motor->pid.ctrl_signal = -motor->max_speed;
}

void set_speed(Motor *motor){
	/*if (abs(motor->pid.e_prop) < motor->pid.critical_delta) {
		motor->dynamic_max_speed = motor->min_speed + (abs(motor->pid.e_prop) * (motor->max_speed - motor->min_speed)) / motor->pid.critical_delta;
	} else {
		motor->dynamic_max_speed = motor->max_speed;
	}

	// Clamp PID output to speed cap
	if (motor->pwm_value > motor->dynamic_max_speed) motor->pwm_value = motor->dynamic_max_speed;
	else if (motor->pwm_value < -motor->dynamic_max_speed) motor->pwm_value = -motor->dynamic_max_speed;
	*/
	if (motor->pid.ctrl_signal > motor->pwm_value + ACCEL_CONST) {
		motor->pwm_value += ACCEL_CONST;  // Gradual acceleration
	} else if ((motor->pid.ctrl_signal < motor->pwm_value - DECEL_CONST)) {
		motor->pwm_value -= DECEL_CONST;  // Gradual deceleration
	} else {
		motor->pwm_value = motor->pid.ctrl_signal; // Maintain speed
	}
}

void update_stall(Motor *motor){
	/*
	uint64_t now = micros();
	if (now - motor->pid.prev_stall_time >= CURRENT_CHECK_MICROS) {
		int64_t delta = abs(motor->pid.current_pos - motor->pid.last_pos);

		if (motor->current_draw > CURRENT_LIMIT && delta < ENCODER_TICK_INTERVAL) {
			if (motor->pwm_value > 0) motor->stall_fwd		= 1;
			else if (motor->pwm_value < 0) motor->stall_bwd	= 1;
		}

		motor->pid.prev_stall_time	= now;
		motor->pid.last_pos			= motor->pid.current_pos;
	}
	*/
	if (motor->current_draw > CURRENT_LIMIT) {
		if (motor->moving_fwd) motor->stall_fwd			= 1;
		else if (motor->moving_bwd) motor->stall_bwd	= 1;
	}

	// Reset stall if switching direction
	if (motor->moving_bwd && motor->stall_fwd) motor->stall_fwd = 0;
	if (motor->moving_fwd && motor->stall_bwd) motor->stall_bwd = 0;
}

void move_abs(Motor *motor, volatile int32_t setpoint, volatile int32_t currentpos, float current){
	motor->pid.current_pos	= currentpos;
	motor->pid.setpoint		= setpoint;
	motor->current_draw		= current;

	calculatePID(motor);

	//if(abs(motor->pid.ctrl_signal) > motor->pid.critical_delta)
	//	motor->pwm_value = motor->max_speed;
	//else
	//	motor->pwm_value = ((abs(motor->pid.ctrl_signal)/motor->pid.critical_delta)*motor->max_speed);

	set_speed(motor);

	update_stall(motor);

	// Check for blocked direction & stop moving in that direction
	if ((motor->pwm_value > 0 && motor->stall_fwd) || (motor->pwm_value < 0 && motor->stall_bwd)) {
		stop(motor);
		return;
	}

	if(motor->motortype == 1) *motor->ocr1 = motor->pwm_value;

	if(motor->pid.ctrl_signal > MIN_POS_DELTA) fwd(motor); // && !fwd_current
	else if(motor->pid.ctrl_signal < -MIN_POS_DELTA) bwd(motor);
	else stop(motor);

	motor->pid.prev_setpoint = motor->pid.setpoint;
}

void set_max_speed(Motor *motor, uint8_t speed){
	motor->max_speed = speed;
}