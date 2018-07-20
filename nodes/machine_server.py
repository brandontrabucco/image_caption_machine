#!/usr/bin/env python


"""Author: Brandon Trabucco
Server directly interacts with the world places.
"""


import rospy


from image_caption_machine.machine import Server


if __name__ == "__main__":
    """Start the server and listen to requests.
    """

    Server()
