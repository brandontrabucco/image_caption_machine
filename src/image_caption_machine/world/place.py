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
from tf.transformations import euler_from_quaternion


from image_caption_machine.msg import WorldPlace
from image_caption_machine.convert.message import convert_ros_message_to_dictionary
from image_caption_machine.convert.message import convert_dictionary_to_ros_message


class Place(object):
    """Utility class for managing physycal naed locations.
    """
    
    def __init__(self, name="default", pose_stamped=PoseStamped(
            Header(0, rospy.Time(secs=0, nsecs=0), "None"),
            Pose(Point(0.0, 0.0, 0.0), 
                 Quaternion(0.0, 0.0, 0.0, 0.0))), 
            x=None, y=None,
            json=None, msg=None):
        """Initialize the class with default parameters.
        Args:
            name: str REQUIRED
            pose_stamped: PoseStamped REQUIRED
            x: float
            y: float
            json: {name: "...", pose_stamped: {...}}
            msg: WorldPlace message
        """

        self.name = name
        self.pose_stamped = pose_stamped

        if x is not None:
            self.pose_stamped.pose.position.x = x
        if y is not None:
            self.pose_stamped.pose.position.y = y

        if json is not None:
            self.json = json
        if msg is not None:
            self.msg = msg


    @property
    def json(self):
        """Serialize the place to json.
        """

        return {"name": self.name, "pose_stamped": 
            convert_ros_message_to_dictionary(self.pose_stamped)}


    @json.setter
    def json(self, val):
        """Load json into the odom object.
        """

        self.name = val["name"]
        self.pose_stamped = convert_dictionary_to_ros_message(
            "geometry_msgs/PoseStamped", val["pose_stamped"])


    @property
    def msg(self):
        """Utility to convert Place() to WorldPlace message.
        """

        return WorldPlace(name=self.name, pose_stamped=self.pose_stamped)


    @msg.setter
    def msg(self, val):
        """Utility to convert WorldPlace message to Place().
        """

        self.name = val.name
        self.pose_stamped = val.pose_stamped


    @property
    def x(self):
        """Helper to get the x position.
        """

        return self.pose_stamped.pose.position.x


    @property
    def y(self):
        """Helper to get the y position.
        """

        return self.pose_stamped.pose.position.y

    
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

