import tensorflow as tf
import cv2
import os

from captioning_utils.handle_request import HandleRequest

class Machine(object):

    def __init__(self):
        self.server_url = "ec2-18-216-95-59.us-east-2.compute.amazonaws.com"
        self.image_uri = (os.path.dirname(os.path.realpath(__file__)) + "image_buffer.jpg")

    def run_machine(self, image):
        cv2.imwrite(self.image_uri, image)
        r = HandleRequest(self.image_uri, self.server_url)
        if r.caption() is not None:
            return r.caption()[0][0]
        else:
            return "encountered a problem."
