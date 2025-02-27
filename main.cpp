#include <iostream>
#include <unistd.h> // for usleep
#include "MPU9255.h"

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
