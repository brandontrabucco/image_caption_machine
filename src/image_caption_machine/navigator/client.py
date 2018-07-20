"""Author: Brandon Trabucco
A utility for requesting captions.
"""


import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped


from image_caption_machine.navigator.helper import Helper
from image_caption_machine.navigator.abstract import Abstract
from image_caption_machine.srv import NavigatorGo
from image_caption_machine.srv import NavigatorFinished
from image_caption_machine.srv import NavigatorAbort


class Client(Abstract):
    """Class for requsting image captions.
    """

    def __init__(self):
        """Build the ROS services.
        """

        rospy.wait_for_service('icm/navigator/go', timeout=10.0)
        self.go_service = rospy.ServiceProxy(
            "icm/navigator/go", NavigatorGo)
        self.callback_subscriber = rospy.Subscriber(
            "icm/navigator/callback", Empty,
            self.handle_callback, queue_size=2)
        rospy.wait_for_service('icm/navigator/finished', timeout=10.0)
        self.finished_service = rospy.ServiceProxy(
            "icm/navigator/finished", NavigatorFinished)
        rospy.wait_for_service('icm/navigator/abort', timeout=10.0)
        self.abort_service = rospy.ServiceProxy(
            "icm/navigator/abort", NavigatorAbort)
        self.fn = (lambda: None)


    def handle_callback(self, m):
        """Execute the callback function.
        """

        self.fn()
        self.fn = (lambda: None)


    def go(self, goal_pose):
        """Start a navigation command
        """

        self.go_service(goal_pose)


    def callback(self, fn):
        """Callback function runs after goal is reached or aborted.
        """

        self.fn = fn


    def finished(self):
    	"""Check if the navigation is finished.
    	"""

    	return self.finished_service().data


    def abort(self):
        """Quickly halt the navigation process.
        """

        self.abort_service(Bool(True))

