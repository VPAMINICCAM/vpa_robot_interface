#!/usr/bin/python3

import rospy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from jetcam.csi_camera import CSICamera

class jetcam:

    def __init__(self) -> None:
        self.camera = CSICamera(width=640,height=320)

        self.bridge = CvBridge()

        self.ima