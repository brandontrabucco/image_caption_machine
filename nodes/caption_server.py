#!/usr/bin/env python


"""Author: Brandon Trabucco
Server that directly requests captions from image bytes.
"""


import rospy


from image_caption_machine.captioner import Server


if __name__ == "__main__":
    """Start the server and listen to requests.
    """

    Server()
