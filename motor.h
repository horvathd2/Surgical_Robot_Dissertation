/*
 * motor.h
 *
 * Created: 10.08.2024 15:38:13
 *  Author: H.Dani
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#include "config.h"

#define MIN_POS_DELTA	0
#define ACCEL_CONST		5
#define DECEL_CONST		5

struct PID{
	int32_t prev_error;
	int32_t current_pos;
	int32_t setpoint;
	float e_prop;
	float e_dot;
	float e_int;
	uint32_t us_time;
	uint32_t d_time;
	uint32_t prev_time;
	float critical_delta;
	float ctrl_signal;
	float Kp;
	float Kd;
	float Ki;
};

typedef struct motor{
	uint8_t motortype;
	volatile uint8_t *ddreg;
	uint8_t ddrpin1;
	uint8_t ddrpin2;
	volatile uint8_t *portdirpin;
	uint8_t dirpin1;
	uint8_t dirpin2;
	volatile uint8_t *ddrpwm;
	uint8_t pwmpin;
	volatile uint16_t *ocr1;
	volatile uint16_t *ocr2;
	int16_t pwm_value;
	uint8_t max_speed;
	volatile int32_t encoder;
	uint16_t ticks_per_rev;
	struct PID pid;
}Motor;

Motor init_motor(uint8_t motortype, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2,
				volatile uint8_t *ddreg, uint8_t ddrpin1, uint8_t ddrpin2, volatile int32_t encoder,
				volatile uint8_t *ddrpwm, uint8_t pwmpin, volatile uint16_t *ocr1,
				volatile uint16_t *ocr2 , uint8_t max_speed);

void init_pid(Motor *motor, float kp, float kd, float ki);

void init_pwm(void);

void fwd(Motor *motor);

void bwd(Motor *motor);

void stop(Motor *motor);

void calculatePID(Motor *motor);

void set_speed(Motor *motor);

void move_abs(Motor *motor, int32_t setpoint, volatile int32_t currentpos);

void home_motor(Motor *motor);

#endif /* MOTOR_H_ */