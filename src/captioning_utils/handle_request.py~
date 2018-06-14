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

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image_uri",
        type=str,
        default="/home/ubuntu/im2txt/im2txt/data/coco/raw-data/val2014/COCO_val2014_000000224477.jpg")
    parser.add_argument(
        "--server_url",
        type=str,
        default="http://ec2-18-216-95-59.us-east-2.compute.amazonaws.com")
    args = parser.parse_args()
    r = HandleRequest(args.image_uri, args.server_url)
    print("Image Caption for %s:"%args.image_uri)
    for i, caption in enumerate(r.caption()):
        print(" %d) %s (p=%f)" % (i, caption[0], caption[1]))
