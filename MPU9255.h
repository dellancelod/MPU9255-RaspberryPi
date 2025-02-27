#ifndef MPU9255_H
#define MPU9255_H

#include <stdint.h>
#include <pigpio.h>
#include "calibration/CalibrationData.h"

// This class wraps MPU9255 operations using pigpio for I2C access.
// It reads accelerometer, gyroscope, and magnetometer data and computes
// pitch, roll and yaw (in degrees).
class MPU9255 {
public:
    // Constructor: default I2C bus (usually 1) and device address (0x68)
    MPU9255(int i2c_bus = 1, int address = 0x68);
    ~MPU9255();

    // Initialize the sensor and required I2C interfaces.
    bool initialize();

    // Test the connection by reading the WHO_AM_I register.
    bool testConnection();

    // Update all sensor data and compute angles.
    void update(const CalibrationData &cal);

    // Latest raw sensor data
    int16_t ax, ay, az;   // Accelerometer
    int16_t gx, gy, gz;   // Gyroscope
    int16_t mx, my, mz;   // Magnetometer

    // Computed angles (in degrees)
    double pitch, roll, yaw;

    // Read individual sensors (returns true if read was successful)
    bool readAccelerometer();
    bool readGyroscope();
    bool readMagnetometer();

    // Compute pitch, roll and yaw from raw sensor values.
    void computeAngles();

private:
    int i2c_bus;
    int i2c_address;
    int i2c_handle;       // Handle for MPU9255
    int mag_i2c_handle;   // Handle for magnetometer (AK8963)
    int mag_address;      // Magnetometer I2C address

    // Read several bytes starting at a given register from the MPU9255.
    bool readRegisters(int reg, int count, uint8_t* dest);
    // Write a single byte to a register.
    bool writeRegister(int reg, uint8_t data);
    // Initialize the magnetometer.
    bool initMagnetometer();
};

#endif // MPU9255_H
