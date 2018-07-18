"""Author: Brandon Trabucco.
Utility class for loading and managing locations in the robot's map.
"""


import json
import math
from rt_msgs.msg import Odom


class Place(object):
    """Utility class for managing physycal naed locations.
    """
    
    def __init__(self, name="default", odom=Odom(
            xPos=0.0, yPos=0.0, yaw=0.0, 
            xVel=0.0, yVel=0.0, 
            xAng=0.0, yAng=0.0, 
            xAngVel=0.0, yAngVel=0.0), x=None, y=None):
        """Initialize the class with default parameters.
        Args:
            name: str
            odom: Odom() object
            x: float
            y: float
        """

        self.name = name
        self.odom = odom
        if x is not None:
            self.odom.xPos = x
        if y is not None:
            self.odom.yPos = y


    @property
    def json(self):
        """Serialize the place to json.
        """

        return {"name": self.name, "odom": {
            "xPos": self.odom.xPos,
            "yPos": self.odom.yPos,
            "yaw": self.odom.yaw,
            "xVel": self.odom.xVel,
            "yVel": self.odom.yVel,
            "xAng": self.odom.xAng,
            "yAng": self.odom.yAng,
            "xAngVel": self.odom.xAngVel,
            "yAngVel": self.odom.yAngVel}}

    @json.setter
    def json(self, val):
        """Load json into the odom object.
        """

        self.odom.xPos = val["xPos"]
        self.odom.yPos = val["yPos"]
        self.odom.yaw = val["yaw"]
        self.odom.xVel = val["xVel"]
        self.odom.yVel = val["yVel"]
        self.odom.xAng = val["xAng"]
        self.odom.yAng = val["yAng"]
        self.odom.xAngVel = val["xAngVel"]
        self.odom.yAngVel = val["yAngVel"]


    @property
    def x(self):
        """Helper to get the x position.
        """

        return self.odom.xPos


    @property
    def y(self):
        """Helper to get the y position.
        """

        return self.odom.yPos

    
    def to(self, other):
        """Helper to get the length to another place.
        Args:
            other: Place() object
        """

        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt((dx * dx) + (dy * dy))

