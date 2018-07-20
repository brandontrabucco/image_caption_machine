"""Author: Brandon Trabucco
Serve the Image Caption Machine on a ROS node.
"""


import rospy


from image_caption_machine.machine.abstract import Abstract
from image_caption_machine.machine.helper import Helper
from image_caption_machine.srv import Machine, MachineResponse


class Server(Abstract):
    """Utility class to serve Image Caption Machine.
    """

    def __init__(self, node_name="machine_server"):
        """Start the API services.
        """

        rospy.init_node(node_name)
        self.helper = Helper()
        self.welcome_service = rospy.Service(
            "icm/machine/welcome", Machine, self.welcome)
        self.navigate_service = rospy.Service(
            "icm/machine/navigate", Machine, self.navigate)
        self.learn_service = rospy.Service(
            "icm/machine/learn", Machine, self.learn)
        self.where_service = rospy.Service(
            "icm/machine/where", Machine, self.where)
        self.caption_service = rospy.Service(
            "icm/machine/caption", Machine, self.caption)
        self.recite_service = rospy.Service(
            "icm/machine/recite", Machine, self.recite)
        self.help_service = rospy.Service(
            "icm/machine/help", Machine, self.help)
        self.stop_service = rospy.Service(
            "icm/machine/stop", Machine, self.stop)
        rospy.spin()


    def welcome(self, m=None):
        """Provide useful info when the app starts.
        """

        return MachineResponse(self.helper.welcome())


    def navigate(self, m=None):
        """The user asked to navigate to a location.
        Example:
            Navigate to the "Office".
        """

        return MachineResponse(self.helper.navigate(m.input))


    def learn(self, m=None):
        """Learn the robots position as the given name.
        Example:
            This location is my "Office".
        """

        return MachineResponse(self.helper.learn(m.input))


    def where(self, m=None):
        """Check the robots map find the closest place.
        Example:
            Where place are you.
        """

        return MachineResponse(self.helper.where())


    def caption(self, m=None):
        """Read an incoming image and process a caption asynchronously.
        Example:
            Look around you.
        """

        return MachineResponse(self.helper.caption())


    def recite(self, m=None):
        """Speak out the latest image caption.
        Example:
            Tell what you saw.
        """

        return MachineResponse(self.helper.recite())


    def help(self, m=None):
        """The user has asked for help using the application.
        Example:
            Help.
        """

        return MachineResponse(self.helper.help())


    def stop(self, m=None):
        """The user has asked to exit the application.
        Example:
            Stop.
        """

        return MachineResponse(self.helper.stop())

