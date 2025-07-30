/*
 * imu.h
 *
 *  Created on: May 5, 2025
 *      Author: sanket
 */

#ifndef IMU_H_
#define IMU_H_

// IMU Definitions
#define IMU_ADDR        0x69    // Address of ICM-20948
#define REG_BANK_SEL    0x7F
#define PWR_MGMT_1      0x06
#define PWR_MGMT_2      0x07
#define INT_PIN_CFG     0x0F
#define ACCEL_XOUT_H    0x2D
#define GYRO_XOUT_H     0x33

static void selectBank(uint8_t bank);
static void imuRead(uint8_t reg, uint8_t *dst, uint8_t n);
bool initIMU(void);
bool readIMU(int16_t *accel, int16_t *gyro);


#endif /* IMU_H_ */
