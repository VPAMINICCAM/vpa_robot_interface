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
The actuators of the chassis is enabled with the launch file

```
roslaunch vpa_robot_interface vpa_traction_start.launch
```

This will enable the steering servo and the traction motors. It will subsrcibes to a 'actuator_cmd' topic, as well as the 'local_brake' and '/global_brake' topic as saftey lock.

A quick test can be performed by launching

```
roslaunch vpa_robot_interface vpa_joystick_operation.launch
```
By pressing the LB and RB buttons at the same time, the local brake will be unlock and a mannul drive is possible when global brake is disabled by a control node.

## Camera
The camera is launched by
```
roslaunch vpa_robot_interface vpa_camera.launch
```
The currently default calibration file is in the config folder.