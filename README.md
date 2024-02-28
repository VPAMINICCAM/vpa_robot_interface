# VPA Robot Car Interface

The package of VPA Car interface aims to contains all sensors, actuators drivers in one package. Thhis package is therefore free of any high-level logic but only performing commands from other sensors.

This package is again branched by the type of robot.

# DB19

The DB19 chassis is originally a duckietwon db19 chassis. Extra tof sensors and closed loop speed controller is implemented based on its coding. On the other hand, despite the well-developed package [here]|(https://github.com/duckietown/dt-duckiebot-interface), the dependcies are across different packages. The VPA MiniCCAM build a light weight package for this chassis for the entry-level students.
## Dependencies
- [ ] containerization of this package

Currently, the dependencies still require manual installment. The package is built based on ROS Noetic, which is python3. The OS of current VPA db19 robot is Ubuntu-mate 20.04 upgraded from 18.04. This is done this way since the 20.04 image is broken.

The hardware of the db19 robot is Raspberry Pi 3B+. The camera and I2C interface is required to be enabled.

## Sensors
The sensor of this chassis including
1. Encoders of each motors, but only one phase of output is transmitted to the Pi. Limited by the Duckietown HAT hardware settings.
2. A 170-degree wide range camera on CSI port.
3. A VL51L1x ToF sensor on I2C bus.

## Actuators
The two motors are driven by the Duckietown HAT, controlled through I2C bus as well. There are originally four RGB Leds that may be used for somewhat purpose, but it is now disconnect due to lack of usages.



