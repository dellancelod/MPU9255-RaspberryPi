#ifndef CALIBRATIONDATA_H
#define CALIBRATIONDATA_H

struct CalibrationData {
    double accel_offset[3];
    double gyro_offset[3];
    double mag_offset[3];
};

#endif