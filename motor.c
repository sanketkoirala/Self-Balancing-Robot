#include <stdint.h>
#include <stdbool.h>
#include "motor.h"
#include "wait.h"
#include "clock.h"
#include "tm4c123gh6pm.h"

void initMotor(void)
{
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1 | SYSCTL_RCGCPWM_R0;   // PWM1 for PF1, PWM0 for PB5
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Port F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2; // Port C
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Port B

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2; // Port C
    _delay_cycles(3);                        // Wait for clock

    GPIO_PORTC_LOCK_R = 0x4C4F434B;          // Unlock Port C
    GPIO_PORTC_CR_R |= 0x30;                 // Allow changes to PC4, PC5
    GPIO_PORTC_DIR_R |= IN3_MASK | IN4_MASK;
    GPIO_PORTC_DEN_R |= IN3_MASK | IN4_MASK;

    _delay_cycles(3); // Allow clocks to stabilize

    // Configure Port F: PF1 (ENA), PF2 (IN1), PF3 (IN2)
    GPIO_PORTF_DIR_R |= ENA_MASK | IN1_MASK | IN2_MASK;
    GPIO_PORTF_DEN_R |= ENA_MASK | IN1_MASK | IN2_MASK;
    GPIO_PORTF_AFSEL_R |= ENA_MASK; // PF1 alternate function
    GPIO_PORTF_PCTL_R &= ~GPIO_PCTL_PF1_M;
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5;

    // Configure Port B: PB5 (ENB)
    GPIO_PORTB_DIR_R |= ENB_MASK;
    GPIO_PORTB_DEN_R |= ENB_MASK;
    GPIO_PORTB_AFSEL_R |= ENB_MASK;
    GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB5_M;
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_M0PWM3;

    // Configure Port C: PC4 (IN3), PC5 (IN4)
    GPIO_PORTC_DIR_R |= IN3_MASK | IN4_MASK;
    GPIO_PORTC_DEN_R |= IN3_MASK | IN4_MASK;

    // Reset PWM modules
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1 | SYSCTL_SRPWM_R0;
    SYSCTL_SRPWM_R = 0;

    // Configure PWM1 Generator 2 → M1PWM5 (Motor 1, PF1)
    PWM1_2_CTL_R = 0;
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    PWM1_2_LOAD_R = 19999;
    PWM1_2_CMPB_R = 7999;
    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R |= PWM_ENABLE_PWM5EN;

    // Configure PWM0 Generator 1 → M0PWM3 (Motor 2, PB5)
    PWM0_1_CTL_R = 0;
    PWM0_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    PWM0_1_LOAD_R = 19999;
    PWM0_1_CMPB_R = 7999;
    PWM0_1_CTL_R = PWM_1_CTL_ENABLE;
    PWM0_ENABLE_R |= PWM_ENABLE_PWM3EN;
}




void rotateForward(void)
{

    IN1 = 1;
    IN2 = 0;

    IN3 = 1;
    IN4 = 0;


    // Gradually increase motor speed (shared PWM)
    int i;
    for (i = 10000; i < 19999; i++)
    {

       PWM1_2_CMPB_R = i;    // Increase duty cycle
       PWM0_1_CMPB_R = i;
       waitMicrosecond(500); // Small delay for smooth acceleration
    }
}

void rotateReverse(void)
{
    IN1 = 0;
    IN2 = 1;

    IN3 = 0;
    IN4 = 1;


    // Gradually increase motor speed (shared PWM)
    int i;
    for (i = 10000; i < 19999; i++)
    {
        PWM1_2_CMPB_R = i;    // Increase duty cycle
        PWM0_1_CMPB_R = i;
        waitMicrosecond(500); // Small delay for smooth acceleration


    }
}



// Set motor speed with direction (positive = forward, negative = backward)
void setMotorSpeed(int speed)
{
    // Determine direction
    if (speed > 0) {
        IN1 = IN3 = 1;
        IN2 = IN4 = 0;
    } else {
        IN1 = IN3 = 0;
        IN2 = IN4 = 1;
        speed = -speed; // Make speed positive
    }

    // Map PID output to PWM range
    uint32_t pwm = BALANCE_PWM + (uint32_t)(speed * 20.0f); // Scale factor

    // Limit PWM
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    if (pwm < MIN_PWM && speed > 0) pwm = MIN_PWM;

    PWM1_2_CMPB_R = pwm;
    PWM0_1_CMPB_R = pwm; // PB5 (Motor 2)

}

// Stop motors
void stopMotors(void)
{
    IN1 = IN3 = 0;
    IN2 = IN4 = 0;

    PWM0_1_CMPB_R = 0;
    PWM1_2_CMPB_R = 0;
}

