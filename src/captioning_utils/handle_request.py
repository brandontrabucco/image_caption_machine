import requests as rq
import tensorflow as tf
import json

class HandleRequest(object):
    
    def __init__(self, image_uri, server_url):
        with tf.gfile.GFile(
                image_uri, "rb") as f:
            self.r = rq.post(
                server_url,
                data=f.read())
        
    def caption(self):
        if self.r.status_code == 200:
            return self.r.json()["image_caption"]
        else:
            return [["", 0.0]]

