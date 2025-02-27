#include "MPU9255.h"
#include <iostream>
#include <unistd.h>   // for usleep
#include <cmath>

// MPU9255 register definitions
#define MPU9255_PWR_MGMT_1      0x6B
#define MPU9255_WHO_AM_I        0x75
#define MPU9255_ACCEL_XOUT_H    0x3B
#define MPU9255_GYRO_XOUT_H     0x43
#define MPU9255_INT_PIN_CFG     0x37

// AK8963 (magnetometer) register definitions
#define AK8963_ADDRESS          0x0C
#define AK8963_ST1              0x02
#define AK8963_HXL              0x03   // Data registers start here
#define AK8963_CNTL1            0x0A   // Control register

MPU9255::MPU9255(int i2c_bus, int address)
: i2c_bus(i2c_bus), i2c_address(address), i2c_handle(-1),
  mag_i2c_handle(-1), mag_address(AK8963_ADDRESS),
  ax(0), ay(0), az(0), gx(0), gy(0), gz(0),
  mx(0), my(0), mz(0), pitch(0), roll(0), yaw(0)
{
}

MPU9255::~MPU9255() {
    if(i2c_handle >= 0) {
        i2cClose(i2c_handle);
    }
    if(mag_i2c_handle >= 0) {
        i2cClose(mag_i2c_handle);
    }
    gpioTerminate();
}

bool MPU9255::initialize() {
    // Initialize pigpio
    if(gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed." << std::endl;
        return false;
    }

    // Open I2C connection to MPU9255
    i2c_handle = i2cOpen(i2c_bus, i2c_address, 0);
    if(i2c_handle < 0) {
        std::cerr << "Failed to open I2C device for MPU9255." << std::endl;
        return false;
    }

    // Wake up MPU9255 (exit sleep mode)
    if(!writeRegister(MPU9255_PWR_MGMT_1, 0x00)) {
        std::cerr << "Failed to wake up MPU9255." << std::endl;
        return false;
    }
    usleep(100000); // wait 100ms

    // Enable bypass mode to access the magnetometer directly
    if(!writeRegister(MPU9255_INT_PIN_CFG, 0x02)) {
        std::cerr << "Failed to enable bypass mode." << std::endl;
        return false;
    }
    usleep(100000);

    // Open I2C connection to the magnetometer (AK8963)
    mag_i2c_handle = i2cOpen(i2c_bus, mag_address, 0);
    if(mag_i2c_handle < 0) {
        std::cerr << "Failed to open I2C device for magnetometer." << std::endl;
        return false;
    }

    // Initialize magnetometer (set continuous measurement mode 2 with 16-bit output)
    if(!initMagnetometer()) {
        std::cerr << "Failed to initialize magnetometer." << std::endl;
        return false;
    }

    return true;
}

bool MPU9255::testConnection() {
    uint8_t who_am_i = 0;
    if(!readRegisters(MPU9255_WHO_AM_I, 1, &who_am_i)) {
        return false;
    }
    // For many MPU9250/9255 devices, WHO_AM_I is expected to be 0x71.
    return (who_am_i == 0x71);
}

bool MPU9255::readRegisters(int reg, int count, uint8_t* dest) {
    int result = i2cReadI2CBlockData(i2c_handle, reg, reinterpret_cast<char*>(dest), count);
    return (result == count);
}

bool MPU9255::writeRegister(int reg, uint8_t data) {
    int result = i2cWriteByteData(i2c_handle, reg, data);
    return (result >= 0);
}

bool MPU9255::readAccelerometer() {
    uint8_t buffer[6];
    if(!readRegisters(MPU9255_ACCEL_XOUT_H, 6, buffer)) {
        return false;
    }
    ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    az = (int16_t)((buffer[4] << 8) | buffer[5]);
    return true;
}

bool MPU9255::readGyroscope() {
    uint8_t buffer[6];
    int result = i2cReadI2CBlockData(i2c_handle, MPU9255_GYRO_XOUT_H, reinterpret_cast<char*>(buffer), 6);
    if(result != 6) {
        return false;
    }
    gx = (int16_t)((buffer[0] << 8) | buffer[1]);
    gy = (int16_t)((buffer[2] << 8) | buffer[3]);
    gz = (int16_t)((buffer[4] << 8) | buffer[5]);
    return true;
}

bool MPU9255::initMagnetometer() {
    // Set magnetometer to continuous measurement mode 2 with 16-bit output.
    int result = i2cWriteByteData(mag_i2c_handle, AK8963_CNTL1, 0x16);
    if(result < 0) {
        return false;
    }
    usleep(10000); // wait 10ms for the magnetometer to settle
    return true;
}

bool MPU9255::readMagnetometer() {
    // Check if data is ready by reading ST1 register.
    int result = i2cReadByteData(mag_i2c_handle, AK8963_ST1);
    if(result < 0) {
        return false;
    }
    uint8_t st1 = (uint8_t)result;
    if(!(st1 & 0x01)) { // data not ready
        return false;
    }
    uint8_t buffer[7];
    result = i2cReadI2CBlockData(mag_i2c_handle, AK8963_HXL, reinterpret_cast<char*>(buffer), 7);
    if(result != 7) {
        return false;
    }
    // Note: The magnetometer returns little-endian data.
    mx = (int16_t)((buffer[1] << 8) | buffer[0]);
    my = (int16_t)((buffer[3] << 8) | buffer[2]);
    mz = (int16_t)((buffer[5] << 8) | buffer[4]);
    // buffer[6] is ST2 (overflow status), which we ignore here.
    return true;
}

void MPU9255::update(const CalibrationData &cal) {
    readAccelerometer();
    readGyroscope();
    readMagnetometer();

    // Apply calibration offsets (convert offsets to the same units as raw readings)
    ax = ax - static_cast<int16_t>(cal.accel_offset[0]);
    ay = ay - static_cast<int16_t>(cal.accel_offset[1]);
    az = az - static_cast<int16_t>(cal.accel_offset[2]);
    
    gx = gx - static_cast<int16_t>(cal.gyro_offset[0]);
    gy = gy - static_cast<int16_t>(cal.gyro_offset[1]);
    gz = gz - static_cast<int16_t>(cal.gyro_offset[2]);
    
    mx = mx - static_cast<int16_t>(cal.mag_offset[0]);
    my = my - static_cast<int16_t>(cal.mag_offset[1]);
    mz = mz - static_cast<int16_t>(cal.mag_offset[2]);

    computeAngles();
}

void MPU9255::computeAngles() {
    // Convert accelerometer raw values to g's.
    // (Assuming ±2g range and sensitivity of 16384 LSB/g)
    double ax_g = ax / 16384.0;
    double ay_g = ay / 16384.0;
    double az_g = az / 16384.0;

    // Compute roll and pitch (in degrees)
    roll = atan2(ay_g, az_g) * 180.0 / M_PI;
    pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / M_PI;

    // Convert magnetometer raw values to microteslas.
    // (Assuming a sensitivity factor of ~0.15 uT/LSB; adjust as needed.)
    double mx_uT = mx * 0.15;
    double my_uT = my * 0.15;
    double mz_uT = mz * 0.15;

    // Tilt-compensate magnetometer readings for yaw calculation.
    double pitch_rad = pitch * M_PI / 180.0;
    double roll_rad  = roll * M_PI / 180.0;

    double magX_comp = mx_uT * cos(pitch_rad) + mz_uT * sin(pitch_rad);
    double magY_comp = mx_uT * sin(roll_rad) * sin(pitch_rad) + my_uT * cos(roll_rad) - mz_uT * sin(roll_rad) * cos(pitch_rad);

    yaw = atan2(-magY_comp, magX_comp) * 180.0 / M_PI;
}
