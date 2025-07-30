// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>


// Bit-banded addresses for Motor 1 direction pins (Port F)
#define IN1 (*((volatile uint32_t *)(0x42000000 + ((0x400253FC - 0x40000000) * 32) + 2 * 4))) // PF2
#define IN2 (*((volatile uint32_t *)(0x42000000 + ((0x400253FC - 0x40000000) * 32) + 3 * 4))) // PF3

// Bit-banded addresses for Motor 2 direction pins (Port C)
#define IN3 (*((volatile uint32_t *)(0x42000000 + ((0x400063FC - 0x40000000) * 32) + 4 * 4))) // PC4
#define IN4 (*((volatile uint32_t *)(0x42000000 + ((0x400063FC - 0x40000000) * 32) + 5 * 4))) // PC5

// Pin masks
#define ENA_MASK 0x02  // PF1 - M1PWM5 (Motor 1)
#define ENB_MASK 0x20  // PB5 - M0PWM3 (Motor 2)
#define IN1_MASK 0x04  // PF2
#define IN2_MASK 0x08  // PF3
#define IN3_MASK 0x10  // PC4
#define IN4_MASK 0x20  // PC5


// Motor control
#define MAX_PWM         18000   // Maximum PWM value
#define MIN_PWM         11000   // Minimum PWM value
#define BALANCE_PWM     14000

// Maximum PWM duty for speed
#define MAX_DUTY 10000

// rotate motor in forward direction for testing and during robot start
// before entering pid control  loop
void rotateForward(void);

// rotate motor in reverse direction for testing and during robot start
// before entering pid control  loop
void rotateReverse(void);

// initialize PWM1 generator & GPIO for both motors
void initMotor(void);

// speed ∈ [−MAX_DUTY..+MAX_DUTY]; sign = direction
void setMotorSpeed(int speed);

// when in balanced mode stop
void stopMotors(void);

// set motor speed
void setMotorSpeed(int speed);

#endif // MOTOR_H
