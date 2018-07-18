"""Author: Brandon Trabucco.
Utility class for loading and managing locations in the robot's map.
"""


import rospy
import json
import math
import os.path


import image_caption_machine
import image_caption_machine.utils
from image_caption_machine.place import Place


class WorldHelper(object):

    def __init__(self):
        self.fname = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), "world_helper.json")
        self.create()


    def dump(self):
        with open(self.fname, "w") as f:
            json.dump(self.content, f)


    def create(self):
        if not os.path.isfile(self.fname):
            with open(self.fname, "w") as f:
                json.dump([Place().json], f)
        self.load()


    def load(self):
        if os.path.isfile(self.fname):
            with open(self.fname, "r") as f:
                self.content = []
                for x in json.load(f):
                    p = Place(name=x["name"])
                    p.json = x["odom"]
                    self.content.append(p)


    def append(self, p):
        self.content.append(p)
        self.dump()


    def __contains__(self, x):
        for y in self.content:
            if y.name == x:
                return y
        return None


    def __str__(self):
        return ", ".join([x.name for x in self.content])


    def __repr__(self):
        return ", ".join([x.name for x in self.content])


    def __len__(self):
        return len(self.content)


    def closest(self, q):
        best_place = None
        best_distance = float("inf")
        for p in self.content:
            current_distance = q.to(p)
            if current_distance < best_distance:
                best_place = p
                best_distance = current_distance
        return best_place, best_distance


