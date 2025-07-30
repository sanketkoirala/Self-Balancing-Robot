#include "tm4c123gh6pm.h"
#include "i2c0.h"
#include "uart0.h"
#include "imu.h"

// Select IMU register bank
static void selectBank(uint8_t bank)
{
    writeI2c0Register(IMU_ADDR, REG_BANK_SEL, bank << 4);
}

// Read multiple bytes from IMU
static void imuRead(uint8_t reg, uint8_t *dst, uint8_t n)
{
    readI2c0Registers(IMU_ADDR, reg, dst, n);
}

// Initialize IMU - returns true if successful
bool initIMU(void)
{
    // Check if IMU responds
    if (!pollI2c0Address(IMU_ADDR)) {
        putsUart0("IMU not responding at address 0x69\r\n");
        return false;
    }

    // Wake up the IMU
    selectBank(0);
    writeI2c0Register(IMU_ADDR, PWR_MGMT_1, 0x01); // Auto select best clock source
    writeI2c0Register(IMU_ADDR, PWR_MGMT_2, 0x00); // Enable all sensors

    // Configure accelerometer and gyro
    selectBank(2);
    writeI2c0Register(IMU_ADDR, 0x14, 0x06); // Accel: ±16g, 50Hz
    writeI2c0Register(IMU_ADDR, 0x00, 0x06); // Gyro: ±250dps, 50Hz

    // Return to bank 0
    selectBank(0);

    return true;
}

// Read accelerometer and gyro data - returns true if successful
bool readIMU(int16_t *accel, int16_t *gyro)
{
    uint8_t buf[12];

    selectBank(0);
    imuRead(ACCEL_XOUT_H, buf, 6);      // Read accelerometer data
    imuRead(GYRO_XOUT_H, buf + 6, 6);   // Read gyro data

    // Check if reads were successful (simple check)
    if (isI2c0Error()) {
        return false;
    }

    // Convert to 16-bit values
    accel[0] = (buf[0] << 8) | buf[1];  // X
    accel[1] = (buf[2] << 8) | buf[3];  // Y
    accel[2] = (buf[4] << 8) | buf[5];  // Z

    gyro[0] = (buf[6] << 8) | buf[7];   // X
    gyro[1] = (buf[8] << 8) | buf[9];   // Y
    gyro[2] = (buf[10] << 8) | buf[11]; // Z

    return true;
}
