"""Author: Brandon Trabucco.
The POST request sent to the AWS Server.
"""


import requests as rq
import tensorflow as tf
import json


class AWSPost(object):
    """Utility class for sending a Post request.
    """
    
    def __init__(self, 
                 image_uri, 
                 server_url):
        """Read image bytes and send the post.
        """

        try:
            with tf.gfile.GFile(
                    image_uri, "rb") as f:
                self.r = rq.post(
                    server_url,
                    data=f.read())
            self.caption = (self.r.json()["image_caption"] 
                if self.r.status_code == 200 else [["", 0.0]])

        except Exception, e:
            self.caption = [["", 0.0]]

