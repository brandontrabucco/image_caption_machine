"""Author: Brandon Trabucco.
Utility class for loading and managing locations in the robot's map.
"""


import json
import math
import rospy
from rt_msgs.msg import Odom
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion


from image_caption_machine.msg import WorldPlace


class Place(object):
    """Utility class for managing physycal naed locations.
    """
    
    def __init__(self, name="default", odom=Odom(
            xPos=0.0, yPos=0.0, yaw=0.0, 
            xVel=0.0, yVel=0.0, 
            xAng=0.0, yAng=0.0, 
            xAngVel=0.0, yAngVel=0.0), x=None, y=None, theta=None,
            json=None, msg=None,
            pose=None, pose_stamped=None):
        """Initialize the class with default parameters.
        Args:
            name: str REQUIRED
            odom: Odom REQUIRED
            x: float
            y: float
            theta: float
            json: {name: "...", odom: {...}}
            msg: WorldPlace message
            pose: Pose message
            pose_stamped: PoseStamped message
        """

        self.name = name
        self.odom = odom

        if x is not None:
            self.odom.xPos = x
        if y is not None:
            self.odom.yPos = y
        if theta is not None:
            self.odom.yaw = theta

        if json is not None:
            self.json = json
        if msg is not None:
            self.msg = msg

        if pose is not None:
            self.pose = pose
        if pose_stamped is not None:
            self.pose_stamped = pose_stamped


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

        self.name = val["name"]
        self.odom.xPos = val["odom"]["xPos"]
        self.odom.yPos = val["odom"]["yPos"]
        self.odom.yaw = val["odom"]["yaw"]
        self.odom.xVel = val["odom"]["xVel"]
        self.odom.yVel = val["odom"]["yVel"]
        self.odom.xAng = val["odom"]["xAng"]
        self.odom.yAng = val["odom"]["yAng"]
        self.odom.xAngVel = val["odom"]["xAngVel"]
        self.odom.yAngVel = val["odom"]["yAngVel"]


    @property
    def pose(self):
        """Calculates the pose msg of this place.
        """

        return Pose(Point(self.x, self.y, 0.0), 
            Quaternion(*quaternion_from_euler(self.odom.xAng, self.odom.yAng, self.theta)))


    @pose.setter
    def pose(self, val):
        """Assigns the pose values to this odom.
        """

        self.odom.xPos = val.position.x
        self.odom.yPos = val.position.y
        roll, pitch, yaw = euler_from_quaternion([
          val.orientation.x, 
          val.orientation.y, 
          val.orientation.z, 
          val.orientation.w])
        self.odom.yaw = yaw
        self.odom.xAng = roll
        self.odom.yAng = pitch


    @property
    def pose_stamped(self):
        """Calculates the stamped pose msg of this place.
        """
        
        stamp = Header(0.0, rospy.Time(0), "map")
        return PoseStamped(stamp, self.pose)


    @pose_stamped.setter
    def pose_stamped(self, val):
        """Assigns the stamped pose to this odom.
        """
        
        self.pose = val.pose


    @property
    def msg(self):
        """Utility to convert Place() to WorldPlace message.
        """

        return WorldPlace(name=self.name, odom=self.odom)


    @msg.setter
    def msg(self, val):
        """Utility to convert WorldPlace message to Place().
        """

        self.name = val.name
        self.odom = val.odom


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


    @property
    def theta(self):
        """Helper to get the yaw position.
        """

        return self.odom.yaw


    @property
    def speed(self):
        """Helper to get the absolute speed of the robot.
        """

        dx = self.odom.xVel
        dy = self.odom.yVel
        return math.sqrt((dx * dx) + (dy * dy))

    
    def to(self, other):
        """Helper to get the length to another place.
        Args:
            other: Place() object
        """

        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt((dx * dx) + (dy * dy))


    def __str__(self):
        """Helper to convert the object to string.
        """

        return self.name

