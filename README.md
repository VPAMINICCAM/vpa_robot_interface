# VPA Robot Car Interface

The package of VPA Car interface aims to contains all sensors, actuators drivers in one package. Thhis package is therefore free of any high-level logic but only performing commands from other sensors.

This package is again branched by the type of robot.

# PiRacer
This branch is built for [PiRacer](https://www.waveshare.com/product/robotics/mobile-robots/raspberry-pi-robots/piracer-pro-ai-kit.htm?sku=18492).
It is a raspberry pi 4b mounted on a latrax rally chassis and power train.
Currently the following sensors are installed 
- [x] Camera
- [ ] IMU
- [ ] Lidar

Despite the waveshare provided a pre-built image, the dependencies have been rebuilt and unnecessary libraries have been removed as the image processing resources are not the main focus of the MiniCCAM lab.
- [ ] A docker image suitable for running on raspberry Pi 5 with its original OS

The enviroment built up will be recorded on VPA interal wiki as a guest user does not need pay any concerns on it.

## Traction start