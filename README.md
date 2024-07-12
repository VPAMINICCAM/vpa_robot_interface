# VPA Robot Car Interface

The package of VPA Car interface aims to contains all sensors, actuators drivers in one package. Thhis package is therefore free of any high-level logic but only performing commands from other sensors.

This package is again branched by the type of robot.

## Jetracer 

The host of jetracer is limited to Jetpack 4.5.1, which is ubuntu 18.04. This verison of system combines both python 2.7 and python 3.6. The ROS version is Melodic, which is Python2 based. This bring changes as the jetracer comes with control interface from a Python 3 based library - Adafruit Servo-kit.

Update 12.07.2024:

The past solution is to replaced certain libraries in the settings and force the script chose python3 as interpreter as suggested in this [post](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674). However, this is not ideal, this solution was replaced by a Python2 I2C PWM dirver, modified from Adafruit libraries.


### Install

#### Camera

The [gscam](http://wiki.ros.org/gscam) is used as the wrapper for driving the camera. The mannul install may be done by

```
sudo apt install ros-melodic-gscam
```
and the camera node is launched by 
```
roslaunch vpa_robot_interface vpa_camera.launch
```
with the topic name <code>csi_camera_0/image_raw</code>

#### Lidar
Please install and maks sure the current user may access the USB port
- [rplidar_ros](http://wiki.ros.org/rplidar)

And please follow the steps to give accesibilty to USB port as in the rplidar wiki. One extra step we take is to
<code>sudo usermod -aG dialout jetson</code>

Quick started by

```
roslanch vpa_robot_interface vpa_lidar.launch
```

#### Actuator

The python library requires here is installed by
```
sudo apt install python-smbus
```
and start the node by
```
roslaunch vpa_robot_interface vpa_traction_start.launch
```

##### Test with joysticks
