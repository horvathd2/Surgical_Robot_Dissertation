/*
 * motor.h
 *
 * Created: 10.08.2024 15:38:13
 *  Author: H.Dani
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#include "config.h"

#define MIN_POS_DELTA 10

struct PID{
	int32_t prev_error;
	int32_t current_pos;
	int32_t setpoint;
	int32_t e_prop;
	int32_t e_dot;
	int32_t e_int;
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
	volatile uint16_t *ocr;
	uint8_t pwm_value;
	uint8_t max_speed;
	volatile int32_t encoder;
	uint16_t ticks_per_rev;
	struct PID pid;
}Motor;

Motor init_motor(uint8_t motortype, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2,
				volatile uint8_t *ddreg, uint8_t ddrpin1, uint8_t ddrpin2, volatile int32_t encoder,
				volatile uint8_t *ddrpwm, uint8_t pwmpin, volatile uint16_t *ocr, uint8_t max_speed);

void init_pid(Motor *motor, float kp, float kd, float ki);

inline void fwd(Motor *motor, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2);

inline void bwd(Motor *motor, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2);

inline void stop(Motor *motor, volatile uint8_t *portdirpin, uint8_t dirpin1, uint8_t dirpin2);

inline void calculatePID(Motor *motor);

void move_abs(Motor *motor, int32_t setpoint, volatile int32_t currentpos);

#endif /* MOTOR_H_ */