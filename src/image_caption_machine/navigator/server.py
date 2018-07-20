"""Author: Brandon Trabucco
A utility for requesting captions.
"""


import rospy
import numpy as np
from std_msgs.msg import Empty


from image_caption_machine.navigator.helper import Helper
from image_caption_machine.navigator.abstract import Abstract
from image_caption_machine.srv import NavigatorGo, NavigatorGoResponse
from image_caption_machine.srv import NavigatorFinished, NavigatorFinishedResponse
from image_caption_machine.srv import NavigatorAbort, NavigatorAbortResponse


class Server(Abstract):
    """Class for requsting image captions.
    """

    def __init__(self):
        """Build the ROS services.
        """

        rospy.init_node("navigator_server")
        self.helper = Helper()
        self.go_service = rospy.Service(
            "icm/navigator/go", NavigatorGo, self.go)
        self.callback_publisher = rospy.Publisher(
            "icm/navigator/callback", Empty, queue_size=2)
        self.finished_service = rospy.Service(
            "icm/navigator/finished", NavigatorFinished, self.finished)
        self.abort_service = rospy.Service(
            "icm/navigator/abort", NavigatorAbort, self.abort)
        rospy.spin()


    def go(self, goal_pose):
        """Start a navigation command
        """

        self.helper.go(goal_pose.goal)
        self.helper.callback(lambda: 
            self.callback_publisher.publish(Empty()))
        return NavigatorGoResponse()


    def callback(self, fn):
        """Callback function runs after goal is reached or aborted.
        """

        pass


    def finished(self, m):
    	"""Check if the navigation is finished.
    	"""

    	return NavigatorFinishedResponse(self.helper.finished())


    def abort(self, m):
        """Quickly halt the navigation process.
        """

        if self.data.data:
            self.helper.abort()
        return NavigatorAbortResponse()

