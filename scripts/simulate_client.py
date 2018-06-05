#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from image_caption_machine.srv import Caption

class SimulateClient(object):

    def __init__(self):
        rospy.init_node("simulate_client_node")
        message = rospy.wait_for_message("sensors/rgb/image_raw", Image)
        self.handle_image(message)

    def handle_image(self, image):
        rospy.wait_for_service('caption_service')
        try:
            caption = rospy.ServiceProxy('caption_service', Caption)
            response = caption([0, 0, 0, 0])
            print(response.caption_text)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    sc = SimulateClient()