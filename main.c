#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "i2c0.h"
#include "uart0.h"
#include "wait.h"
#include "clock.h"
#include "motor.h"
#include "imu.h"



#define MAX_CORRECTION  20.0f

// Constants for IMU and control
#define GYRO_SENSITIVITY 131.0f // LSB/(deg/s) for ±250dps range
#define ACCEL_SENSITIVITY 16384.0f // LSB/g for ±2g range
#define RAD_TO_DEG      57.29578f
#define DEG_TO_RAD      0.0174533f
#define SAMPLE_TIME_MS  5       // 5ms sample time (200Hz)
#define ALPHA           0.85f   // Complementary filter coefficient

 // Base PWM for balancing
#define DEADZONE        2.0f

// PID constants
float Kp = 8.0f;    // Proportional gain
float Ki = 0.3f;    // Integral gain
float Kd = 0.8f;    // Derivative gain

// Global variables
float pitch_angle = 0.0f;
float pitch_rate = 0.0f;
float pitch_offset = 0.0f;
float pid_output = 0.0f;

float error = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float prev_error = 0.0f;


// Function prototypes
float calculatePitch(int16_t ax, int16_t ay, int16_t az);
void complementaryFilter(float accel_angle, float gyro_rate);
void balanceControl(void);

int main(void)
{
    int16_t accel[3] = {0}, gyro[3] = {0};
    float accel_angle = 0.0f;

    // Initialize system
    initSystemClockTo40Mhz();
    initI2c0();
    initUart0();
    setUart0BaudRate(115200, 40000000);
    initMotor();

    putsUart0("\r\nSelf-Balancing Robot Initializing...\r\n");

    // Initialize IMU
    if (!initIMU()) {
        putsUart0("ERROR: IMU initialization failed!\r\n");
        while(1);
    }
    waitMicrosecond(100000);

    // Calibration - read IMU for 1 second to find offsets
    putsUart0("Calibrating IMU...\r\n");
    int32_t gyro_sum = 0;
    uint8_t cal_samples = 0;
int i = 0;
    for ( i = 0; i < 100; i++) {
        if (readIMU(accel, gyro)) {
            gyro_sum += gyro[1]; // Y-axis for pitch rate
            cal_samples++;

            // Display raw data during calibration
            putsUart0("Calibrating - Accel: ");
            putIntUart0(accel[1]); putsUart0(" Gyro: ");
            putIntUart0(gyro[1]); putsUart0("\r\n");
        }
        waitMicrosecond(10000);
    }

    if (cal_samples == 0) {
        putsUart0("ERROR: No IMU readings during calibration!\r\n");
        while(1);
    }

    pitch_offset = gyro_sum / (float)cal_samples;
    putsUart0("Calibration complete. Offset: ");
    putIntUart0((int32_t)pitch_offset);
    putsUart0("\r\n");

    // Initial movement sequence
    putsUart0("Starting movement sequence...\r\n");

    // Forward for 3 seconds
    putsUart0("Moving forward...\r\n");
    setMotorSpeed(12000);
    waitMicrosecond(3000000);

    // Backward for 3 seconds
    putsUart0("Moving backward...\r\n");
    setMotorSpeed(-12000);
    waitMicrosecond(3000000);

    stopMotors();
    putsUart0("Entering balancing mode...\r\n");

    // Main control loop

    while (1) {
        // Read IMU data
        if (!readIMU(accel, gyro)) {
            putsUart0("ERROR: IMU read failed!\r\n");
            waitMicrosecond(100000);
            continue;
        }

        // Calculate pitch angle from accelerometer
        accel_angle = calculatePitch(accel[0], accel[1], accel[2]);

        // Calculate pitch rate from gyro (subtract offset)
        pitch_rate = (gyro[1] - pitch_offset) / GYRO_SENSITIVITY;

        // Apply complementary filter
        complementaryFilter(accel_angle, pitch_rate);

        // Run PID control
        balanceControl();

        // Display data via UART
        putsUart0("P: ");
        putFloatUart0(pitch_angle, 1);
        putsUart0("° R: ");
        putFloatUart0(pitch_rate, 1);
        putsUart0("°/s PID: ");
        putFloatUart0(pid_output, 1);
        putsUart0(" PWM: ");
        putIntUart0(PWM1_2_CMPB_R);
        putsUart0("\r\n");

        waitMicrosecond(2000);

    }
}



// Calculate pitch angle from accelerometer data
float calculatePitch(int16_t ax, int16_t ay, int16_t az)
{
    // Convert to g's
    float ax_g = ax / ACCEL_SENSITIVITY;
    float ay_g = ay / ACCEL_SENSITIVITY;
    float az_g = az / ACCEL_SENSITIVITY;

    // Calculate pitch angle in radians
    float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g));

    // Convert to degrees
    return pitch * RAD_TO_DEG;
}

// Complementary filter to combine accel and gyro data
void complementaryFilter(float accel_angle, float gyro_rate)
{
    // Integrate gyro rate to get angle
    pitch_angle = ALPHA * (pitch_angle + gyro_rate * (SAMPLE_TIME_MS / 1000.0f)) +
                 (1.0f - ALPHA) * accel_angle;
}

// PID control for balancing
void balanceControl(void)
{
    // Calculate error (we want pitch_angle = 0)
    error = -pitch_angle;

    // If within deadzone, stop motors and reset integral
    if (fabs(error) < DEADZONE) {
        stopMotors();
        integral *= 0.5f; // Faster decay of integral term
        prev_error = 0;
        pid_output = 0;
        return;
    }

    // Calculate integral term (with anti-windup)
    integral += error * (SAMPLE_TIME_MS / 1000.0f);

    // Stronger limits on integral term
    float max_integral = 10.0f / Ki; // Smaller limit
    if (integral > max_integral) integral = max_integral;
    if (integral < -max_integral) integral = -max_integral;

    // Calculate derivative term (filtered)
    derivative = 0.7f * derivative + 0.3f * ((error - prev_error) / (SAMPLE_TIME_MS / 1000.0f));
    prev_error = error;

    // Calculate PID output
    pid_output = Kp * error + Ki * integral + Kd * derivative;

    // Apply non-linear response for small corrections
    if (fabs(error) < 5.0f) {
        pid_output *= 0.3f; // Reduce response for small tilts
    }

    // Strictly limit PID output to prevent over-correction
    if (pid_output > MAX_CORRECTION) pid_output = MAX_CORRECTION;
    if (pid_output < -MAX_CORRECTION) pid_output = -MAX_CORRECTION;

    // Apply output to motors
    setMotorSpeed((int)pid_output);
}








