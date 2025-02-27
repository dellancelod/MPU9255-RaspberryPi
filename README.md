# MPU9255-RaspberyPi
Simple minimal functioning example of MPU-9255 for Raspbery Pi using **pigpio** library.

# General Information
I've bought WaveShare's 10 DOF IMU Sensor (C) but couldn't make MPU9255 display position data due to developers have not provided a working example. MPU9255 is simply MPU9250 with swaped magnetometer (AK8975 instead of AK8963). If you want to use BMP180 functionality you can refer to https://www.waveshare.com/wiki/10_DOF_IMU_Sensor_%28D%29 for working example, because this version uses the same sensor.

# Connection

| MPU9255  | Raspberry Pi | Function |
| ------------- | ------------- |  ------------- |
| VCC  | 3.3v or 5v* | Power supply voltage  | 
| GND  | GND | Power supply ground  |
| SDA  | SDA | I2C data line  |
| SCL  | SCL | I2C clock line  |
| INT  | - | Interrupt digital output  |
| FSYNC  | GND** | Frame synchronisaion digital input  |

\* MPU9255 requires 3.3V but some modules have 3.3V regulator built in.

** Connect to ground if unused.

# Installation

1. Install pigpio (if not already installed):
```
sudo apt-get update
sudo apt-get install pigpio
```
2. Clone the repo
```
git clone https://github.com/dellancelod/MPU9255-RaspberyPi
```
3. Compile the Code

Use g++ and link against the pigpio, rt, and pthread libraries:
```
cd MPU9255-RaspberyPi
g++ -o mpu9255 main.cpp MPU9255.cpp -lpigpio -lrt -lpthread
```
4. Run the Program

Since pigpio requires elevated privileges, run the executable with sudo:
```
sudo ./mpu9255
```
4.Output
The program will continuously print the pitch, roll, and yaw values to the terminal
![image](https://github.com/user-attachments/assets/d85f37bb-86fb-49c4-93c8-63bcf8e5db6c)

# Calibration
For propper functionality sensor should be calibrated, both acc/gyro and magnetometer offsets saved in a file and further used in a code.

To do this we should compile and run separate script.

1. Compile the code

```
cd calibration
g++ -o calibrate_imu calibrate_imu.cpp ../MPU9255.cpp -lpigpio -lrt -lpthread
```
2. Run the executable with **sudo**
```
sudo ./calibrate_imu
```
Follow the instructions in terminal. Offsets will be saved to calibration.txt and loaded in program.


