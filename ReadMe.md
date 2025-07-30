# Self-Balancing Robot

A sophisticated self-balancing robot implementation using the Texas Instruments TM4C123GH6PM microcontroller and ICM-20948 IMU sensor. This project demonstrates advanced control systems, PID control algorithms, and real-time sensor fusion for autonomous balance maintenance.

## 🚀 Features

- **Real-time Balance Control**: Advanced PID control system for maintaining robot balance
- **Sensor Fusion**: Complementary filter implementation combining accelerometer and gyroscope data
- **High-Frequency Control Loop**: 200Hz control frequency (5ms sample time) for responsive balance
- **UART Debug Interface**: Real-time monitoring and debugging capabilities
- **Modular Design**: Clean, well-organized codebase with separate modules for different functionalities

## 🛠️ Hardware Requirements

### Core Components
- **Microcontroller**: Texas Instruments TM4C123GH6PM
- **IMU Sensor**: ICM-20948 (6-axis motion sensor)
- **Motors**: Dual DC motors with H-bridge control
- **Power Supply**: 3.3V regulated power supply
- **Debug Interface**: UART connection for monitoring

### Pin Configuration
- **Motor 1**: 
  - Direction: PF2 (IN1), PF3 (IN2)
  - PWM: PF1 (M1PWM5)
- **Motor 2**:
  - Direction: PC4 (IN3), PC5 (IN4)
  - PWM: PB5 (M0PWM3)
- **I2C Interface**: 
  - SCL: PB2
  - SDA: PB3
- **UART Debug**: 
  - TX: PA1
  - RX: PA0

## 📁 Project Structure

```
Self Balancing Robot/
├── main.c              # Main application logic and control loop
├── motor.c/h           # Motor control and PWM management
├── imu.c/h             # IMU sensor interface and data processing
├── i2c0.c/h            # I2C communication protocol
├── uart0.c/h           # UART debug interface
├── gpio.c/h            # General purpose I/O management
├── clock.c/h           # System clock configuration
├── wait.c/h            # Timing and delay functions
├── targetConfigs/      # TI Code Composer Studio configuration
└── Debug/              # Build output directory
```

## 🔧 Software Architecture

### Core Modules

#### 1. **Main Control System** (`main.c`)
- PID control algorithm implementation
- Complementary filter for sensor fusion
- Real-time balance control loop
- System initialization and calibration

#### 2. **Motor Control** (`motor.c/h`)
- PWM generation for motor speed control
- H-bridge direction control
- Motor speed and direction management
- Safety features and limits

#### 3. **IMU Interface** (`imu.c/h`)
- ICM-20948 sensor communication
- Accelerometer and gyroscope data processing
- Sensor calibration and offset compensation
- Raw data to angle conversion

#### 4. **Communication** (`i2c0.c/h`, `uart0.c/h`)
- I2C protocol for IMU communication
- UART interface for debugging and monitoring
- Real-time data transmission

### Control Algorithm

The robot uses a sophisticated PID (Proportional-Integral-Derivative) control system for maintaining balance. This control algorithm continuously monitors the robot's pitch angle and applies corrective motor commands to maintain upright position.

#### PID Control Theory
The PID controller calculates the control output using three components:

**Proportional (P)**: Responds to current error
- **Formula**: `P_output = Kp × error`
- **Purpose**: Provides immediate response to deviation from setpoint
- **Effect**: Higher Kp = faster response but potential overshoot

**Integral (I)**: Accumulates past errors
- **Formula**: `I_output = Ki × ∫error dt`
- **Purpose**: Eliminates steady-state error
- **Effect**: Higher Ki = eliminates offset but can cause oscillation

**Derivative (D)**: Predicts future error
- **Formula**: `D_output = Kd × d(error)/dt`
- **Purpose**: Dampens oscillations and improves stability
- **Effect**: Higher Kd = better stability but can amplify noise

#### Current PID Parameters
- **Proportional Gain (Kp)**: 8.0 - Provides strong immediate response
- **Integral Gain (Ki)**: 0.3 - Eliminates steady-state error
- **Derivative Gain (Kd)**: 0.8 - Dampens oscillations and noise
- **Sample Time**: 5ms (200Hz control frequency)
- **Complementary Filter Coefficient**: 0.85 - Balances accelerometer and gyroscope data

#### Control Loop Implementation
```c
// PID control calculation
error = target_angle - current_angle;
integral += error * dt;
derivative = (error - prev_error) / dt;
pid_output = Kp * error + Ki * integral + Kd * derivative;
prev_error = error;
```

#### Sensor Fusion
The system uses a complementary filter to combine accelerometer and gyroscope data:
- **Accelerometer**: Provides absolute angle reference but is noisy
- **Gyroscope**: Provides smooth rate data but drifts over time
- **Filter**: `angle = α × (angle + gyro_rate × dt) + (1-α) × accel_angle`
- **Coefficient α**: 0.85 (85% gyroscope, 15% accelerometer)

#### Balance Control Features
- **Deadzone**: ±2° tolerance to prevent oscillation
- **Output Limiting**: PWM constrained to 11,000-18,000 range
- **Anti-Windup**: Integral term limited to prevent saturation
- **Real-time Tuning**: Parameters can be adjusted via UART interface

## 🚀 Getting Started

### Prerequisites
- Texas Instruments Code Composer Studio (CCS)
- TM4C123GH6PM LaunchPad or compatible board
- ICM-20948 IMU sensor
- Dual DC motors with H-bridge
- Power supply (3.3V)

### Installation

1. **Clone the Repository**
   ```bash
   git clone [repository-url]
   cd Self-Balancing-Robot
   ```

2. **Open in Code Composer Studio**
   - Launch CCS
   - Import the project
   - Configure target settings in `targetConfigs/`

3. **Hardware Setup**
   - Connect IMU sensor to I2C pins (PB2, PB3)
   - Connect motors to specified PWM and direction pins
   - Connect UART debug interface (PA0, PA1)
   - Ensure proper power supply connections

4. **Build and Flash**
   - Build the project in CCS
   - Flash to TM4C123GH6PM
   - Monitor output via UART at 115200 baud rate

### Calibration Process

The robot performs automatic calibration on startup:
1. **IMU Calibration**: 1-second calibration period to determine sensor offsets
2. **Gyroscope Offset**: Calculates and stores gyroscope bias
3. **System Ready**: Robot enters balance control mode

## 📊 Performance Specifications

- **Control Frequency**: 200Hz
- **Sensor Update Rate**: 200Hz
- **Balance Accuracy**: ±2° deadzone
- **PWM Range**: 11,000 - 18,000 (motor control)
- **UART Baud Rate**: 115,200
- **I2C Clock**: 400kHz

## 🔍 Debugging and Monitoring

### UART Debug Interface
The robot provides real-time debugging information via UART:
- Calibration progress and results
- Real-time sensor readings
- PID control parameters
- Balance status and motor commands

### Monitoring Commands
Connect to UART interface to monitor:
- Raw accelerometer and gyroscope data
- Calculated pitch angle and rate
- PID control output
- Motor speed and direction

## 🛡️ Safety Features

- **Motor Limits**: PWM values constrained to safe operating range
- **Deadzone Protection**: Prevents oscillation around balance point
- **Error Handling**: Graceful handling of sensor communication failures
- **Calibration Validation**: Ensures proper sensor operation before control

## 🔧 Configuration

### PID Tuning
Adjust control parameters in `main.c`:
```c
float Kp = 8.0f;    // Proportional gain
float Ki = 0.3f;    // Integral gain  
float Kd = 0.8f;    // Derivative gain
```

### Sensor Configuration
IMU settings in `imu.h`:
```c
#define IMU_ADDR        0x69    // I2C address
#define GYRO_SENSITIVITY 131.0f  // LSB/(deg/s)
#define ACCEL_SENSITIVITY 16384.0f // LSB/g
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License.