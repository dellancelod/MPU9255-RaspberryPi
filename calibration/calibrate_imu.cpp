// calibrate.cpp
#include "../MPU9255.h"
#include "CalibrationData.h"
#include <iostream>
#include <fstream>
#include <unistd.h>


int main() {
    MPU9255 mpu;  // uses default I2C bus (1) and address (0x68)
    if (!mpu.initialize()) {
        std::cerr << "Failed to initialize MPU9255." << std::endl;
        return -1;
    }
    if (!mpu.testConnection()) {
        std::cerr << "MPU9255 connection test failed." << std::endl;
        return -1;
    }
    
    CalibrationData cal;
    
    // ----- Phase 1: Accelerometer and Gyroscope Calibration -----
    std::cout << "Phase 1: Accelerometer and Gyroscope Calibration\n";
    std::cout << "Please hold the device completely still.\n";
    std::cout << "Press Enter to start accelerometer/gyro calibration...";
    std::cin.ignore(); // Wait for the user to press Enter

    const int samples_acc_gyro = 1000;
    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    for (int i = 0; i < samples_acc_gyro; i++) {
        mpu.update();  // read sensor data
        
        // Accumulate accelerometer and gyro readings
        ax_sum += mpu.ax;
        ay_sum += mpu.ay;
        az_sum += mpu.az;
        gx_sum += mpu.gx;
        gy_sum += mpu.gy;
        gz_sum += mpu.gz;
        
        usleep(5000); // 5ms delay between samples
    }
    
    // Compute average offsets for accelerometer and gyro
    cal.accel_offset[0] = ax_sum / samples_acc_gyro;
    cal.accel_offset[1] = ay_sum / samples_acc_gyro;
    cal.accel_offset[2] = az_sum / samples_acc_gyro;
    cal.gyro_offset[0]  = gx_sum / samples_acc_gyro;
    cal.gyro_offset[1]  = gy_sum / samples_acc_gyro;
    cal.gyro_offset[2]  = gz_sum / samples_acc_gyro;
    
    std::cout << "Accelerometer and Gyroscope calibration complete.\n\n";
    
    // ----- Phase 2: Magnetometer Calibration -----
    std::cout << "Phase 2: Magnetometer Calibration\n";
    std::cout << "Slowly rotate the device in all directions.\n";
    std::cout << "Press Enter to start magnetometer calibration...";
    std::cin.ignore(); // Wait for the user to press Enter again

    const int samples_mag = 1000;
    double mx_min = 1e9, my_min = 1e9, mz_min = 1e9;
    double mx_max = -1e9, my_max = -1e9, mz_max = -1e9;
    
    for (int i = 0; i < samples_mag; i++) {
        mpu.update();  // read sensor data
        
        // Update min and max magnetometer readings
        if (mpu.mx < mx_min) mx_min = mpu.mx;
        if (mpu.my < my_min) my_min = mpu.my;
        if (mpu.mz < mz_min) mz_min = mpu.mz;
        if (mpu.mx > mx_max) mx_max = mpu.mx;
        if (mpu.my > my_max) my_max = mpu.my;
        if (mpu.mz > mz_max) mz_max = mpu.mz;
        
        usleep(5000); // 5ms delay between samples
    }
    
    // Compute magnetometer offsets (simple hard-iron correction)
    cal.mag_offset[0] = (mx_min + mx_max) / 2.0;
    cal.mag_offset[1] = (my_min + my_max) / 2.0;
    cal.mag_offset[2] = (mz_min + mz_max) / 2.0;
    
    std::cout << "Magnetometer calibration complete.\n";
    
    // ----- Save Calibration Data -----
    std::ofstream outfile("calibration.txt");
    if (!outfile) {
        std::cerr << "Failed to open calibration file for writing." << std::endl;
        return -1;
    }
    // Write offsets: first line for accel, second for gyro, third for magnetometer
    outfile << cal.accel_offset[0] << " " << cal.accel_offset[1] << " " << cal.accel_offset[2] << std::endl;
    outfile << cal.gyro_offset[0]  << " " << cal.gyro_offset[1]  << " " << cal.gyro_offset[2]  << std::endl;
    outfile << cal.mag_offset[0]   << " " << cal.mag_offset[1]   << " " << cal.mag_offset[2]   << std::endl;
    outfile.close();
    
    std::cout << "Calibration complete. Offsets saved to calibration.txt" << std::endl;
    return 0;
}
