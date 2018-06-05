#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

class SimulateCamera(object):

    def __init__(self):
        rospy.init_node("simulate_camera_node")
        self.publisher = rospy.Publisher("sensors/rgb/image_raw", Image, queue_size=10)
        self.rate = rospy.Rate(29)
        self.h = Header()
        self.h.seq = 0
        self.h.stamp = rospy.get_rostime()
        self.h.frame_id = "usb_cam"
        self.image = Image()
        self.image.header = self.h
        self.image.height = 10
        self.image.width = 10
        self.image.encoding = "rgb8"
        self.image.is_bigendian = 0
        self.image.step = 10
        self.image.data = [125 for i in range(100)]
        self.spin()

    def spin(self):
        print("Starting camera simulation.")
        while not rospy.is_shutdown():
            self.publisher.publish(self.image)
            self.rate.sleep()

if __name__ == "__main__":
    sc = SimulateCamera()