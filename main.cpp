#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h> // for usleep
#include "MPU9255.h"


struct CalibrationData {
    double accel_offset[3];
    double gyro_offset[3];
    double mag_offset[3];
};

bool loadCalibration(const std::string &filename, CalibrationData &cal) {
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Failed to open calibration file: " << filename << std::endl;
        return false;
    }
    infile >> cal.accel_offset[0] >> cal.accel_offset[1] >> cal.accel_offset[2];
    infile >> cal.gyro_offset[0]  >> cal.gyro_offset[1]  >> cal.gyro_offset[2];
    infile >> cal.mag_offset[0]   >> cal.mag_offset[1]   >> cal.mag_offset[2];
    return true;
}

int main() {
    MPU9255 mpu;

    if(!mpu.initialize()){
        std::cerr << "Failed to initialize MPU9255." << std::endl;
        return -1;
    }
    if(!mpu.testConnection()){
        std::cerr << "MPU9255 not connected properly." << std::endl;
        return -1;
    }
    std::cout << "MPU9255 initialized successfully." << std::endl;

    CalibrationData cal;
    if (!loadCalibration("calibration/calibration.txt", cal)) {
        std::cerr << "Calibration data not found. Run the calibration script first." << std::endl;
        return -1;
    }

    // Main loop: update sensor data and print computed angles.
    while(true){
        mpu.update();
        std::cout << "Pitch: " << mpu.pitch 
                  << "  Roll: " << mpu.roll 
                  << "  Yaw: " << mpu.yaw << std::endl;
        usleep(100000); // 100 ms delay
    }

    return 0;
}
