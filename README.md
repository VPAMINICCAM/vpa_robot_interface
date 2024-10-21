# VPA Robot Car Interface

The package of VPA Car interface aims to contains all sensors, actuators drivers in one package. Thhis package is therefore free of any high-level logic but only performing commands from other sensors.

This package is again branched by the type of robot.

# DB19_VPA modified
This branch is built for [Modified DB19]().

Currently the following sensors are installed 
- [x] Camera
- [x] IMU
- [x] Lidar

The enviroment built up will be recorded on VPA internal wiki as a guest user does not need pay any concerns on it.

# vpa_piracer_hat
A self-designed raspberry hat is placed in between raspberry Pi and the PCA9685 chip. An encoder is installed on the motor so that a close-loop speed control is the desired target of such implementations

- [ ] Link the open-source materials of the hat

There is a STM32F103RCT6 MCU on it, and for Piracer, it enabled a encoder channel and USART communication, so that it serves as a lower level control for wheel speed control.

