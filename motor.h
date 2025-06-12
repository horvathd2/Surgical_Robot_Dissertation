/*
 * motor.h
 *
 * Created: 10.08.2024 15:38:13
 *  Author: H.Dani
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#include "config.h"

#define MIN_POS_DELTA			50
#define ACCEL_CONST				2
#define DECEL_CONST				2
#define CURRENT_LIMIT			5.0
#define CURRENT_CHECK_MICROS	50000
#define ENCODER_TICK_INTERVAL	3

#define X_THRESHOLD 160
#define Y_THRESHOLD 400

struct PID{
	int32_t prev_error;
	volatile int32_t current_pos;
	int32_t last_pos;
	volatile int32_t setpoint;
	volatile int32_t prev_setpoint;
	float e_prop;
	float e_dot;
	float e_int;
	uint64_t us_time;
	uint64_t d_time;
	uint64_t prev_time;
	uint64_t prev_stall_time;
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
	uint8_t max_speed;
	uint8_t min_speed;
	uint8_t dynamic_max_speed;
	uint8_t stall_fwd;
	uint8_t stall_bwd;
	uint8_t moving_fwd;
	uint8_t moving_bwd;
	float current_draw;
	volatile uint16_t *ocr1;
	volatile uint16_t *ocr2;
	uint16_t ticks_per_rev;
	int16_t pwm_value;
	struct PID pid;
}Motor;

Motor init_motor(uint8_t motortype, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2,
				volatile uint8_t *ddreg, uint8_t ddrpin1, uint8_t ddrpin2,
				volatile uint8_t *ddrpwm, uint8_t pwmpin, volatile uint16_t *ocr1,
				volatile uint16_t *ocr2 , uint8_t max_speed);

void init_pid(Motor *motor, float kp, float kd, float ki);

void init_pwm(void);

void fwd(Motor *motor);

void bwd(Motor *motor);

void stop(Motor *motor);

void calculatePID(Motor *motor);

void set_speed(Motor *motor);

void update_stall(Motor *motor);

void move_abs(Motor *motor, volatile int32_t setpoint, volatile int32_t currentpos, float current);

void set_max_speed(Motor *motor, uint8_t speed);

#endif /* MOTOR_H_ */