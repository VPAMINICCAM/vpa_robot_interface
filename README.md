# VPA Robot Car Interface

The package of VPA Car interface aims to contains all sensors, actuators drivers in one package. Thhis package is therefore free of any high-level logic but only performing commands from other sensors.

This package is again branched by the type of robot.

## Jetracer 

The host of jetracer is limited to Jetpack 4.5.1, which is ubuntu 18.04. This verison of system combines both python 2.7 and python 3.6. The ROS version is Melodic, which is Python2 based. This bring changes as the jetracer comes with control interface from a Python 3 based library - Adafruit Servo-kit.

On the other hand, the attempt to move all ROS in conatiner fails as accessing the GPIOs inside the conatiner bring extra challenges and remain unsolved at this point.

**To start the test, we plan to put this modified Melodic version interface. And it is strongly recommended to run every node else (not hardware connected) in docker.**

