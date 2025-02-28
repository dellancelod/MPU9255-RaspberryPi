#ifndef MPU9255_H
#define MPU9255_H

#include <stdint.h>
#include <pigpio.h>
#include "calibration/CalibrationData.h"

// MPU9255 register definitions
#define MPU2955_ADDRESS             0x68
#define MPU9255_PWR_MGMT_1          0x6B
#define MPU9255_PWR_MGMT_2          0x6C
#define MPU9255_SIGNAL_PATH_RESET   0x68
#define MPU9255_INT_PIN_CFG        0x37
#define MPU9255_ACCEL_CONFIG        0x1C
#define MPU9255_ACCEL_CONFIG_2      0x1D
#define MPU9255_MOT_DETECT_CTRL     0x69
#define MPU9255_WOM_THR             0x1F
#define MPU9255_GYRO_CONFIG         0x1B
#define MPU9255_CONFIG              0x1A
#define MPU9255_SMPLRT_DIV          0x19
#define MPU9255_INT_ENABLE          0x38
#define MPU9255_INT_STATUS          0x3A
#define MPU9255_WHO_AM_I            0x75

// Gyroscope offset
#define XG_OFFSET_H                 0x13
#define XG_OFFSET_L                 0x14
#define YG_OFFSET_H                 0x15
#define YG_OFFSET_L                 0x16
#define ZG_OFFSET_H                 0x17
#define ZG_OFFSET_L                 0x18

// Accelerometer offset
#define XA_OFFSET_H                 0x77
#define XA_OFFSET_L                 0x78
#define YA_OFFSET_H                 0x7A
#define YA_OFFSET_L                 0x7B
#define ZA_OFFSET_H                 0x7D
#define ZA_OFFSET_L                 0x7E

// AK8963 (magnetometer) register definitions
#define AK8963_ID                   0x00
#define AK8963_ADDRESS              0x0C
#define AK8963_CNTL_1               0x0A   // Control register
#define AK8963_CNTL_2               0x0B
#define AK8963_ST1                  0x02
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

// Data registers
#define AK8963_HXL                  0x03   // magnetometer
#define ACCEL_XOUT_H                0x3B   // accelerometer
#define GYRO_XOUT_H                 0x43   // gyroscope
#define TEMP_OUT_H                  0x41   // thermometer

#define MAGNETOMETER_CAL             0.06    // magnetometer calibration

// This class wraps MPU9255 operations using pigpio for I2C access.
// It reads accelerometer, gyroscope, and magnetometer data and computes
// pitch, roll and yaw (in degrees).
class MPU9255 {
public:
    // Constructor: default I2C bus (usually 1) and device address (0x68)
    MPU9255(int i2c_bus = 1, int address = MPU2955_ADDRESS);
    ~MPU9255();

    // Initialize the sensor and required I2C interfaces.
    bool initialize();

    // Test the connection by reading the WHO_AM_I register.
    bool testConnection();

    // Update all sensor data and compute angles.
    void update(const CalibrationData &cal);
    
    
    // Override function for calibration script
    void update();

    // Latest raw sensor data
    int16_t ax, ay, az;   // Accelerometer
    int16_t gx, gy, gz;   // Gyroscope
    int16_t mx, my, mz;   // Magnetometer

    // Computed angles (in degrees)
    double pitch, roll, yaw;

    bool setAccelRange(uint8_t range); // ranges are: 0 (+-2g), 1 (+-4g), 2 (+-8g), 3 (+-16g)
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

    //magnetometer sensitivity
    double mx_sensitivity;
    double my_sensitivity;
    double mz_sensitivity;

    // accelerometer factory offset
    int AX_offset;
    int AY_offset;
    int AZ_offset;

    //gyroscope factory offset
    int GX_offset;
    int GY_offset;
    int GZ_offset;

    double accel_scale; // Conversion factor: LSB/g

    // Read several bytes starting at a given register from the MPU9255.
    bool readRegisters(int reg, int count, uint8_t* dest);
    // Write a single byte to a register.
    bool writeRegister(int reg, uint8_t data);

    double getMagnetometerSensitivity(int reg);
    int getGyroscopeOffset(int regH, int regL);
    int getAccelerometerOffset(int regH, int regL);
};

#endif // MPU9255_H
