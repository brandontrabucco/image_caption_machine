"""Author: Brandon Trabucco
Serve the Image Caption Machine on a ROS node.
"""


import rospy


from image_caption_machine.srv import Machine
from image_caption_machine.abstract import Abstract


class Client(Abstract):
	"""Utility class to serve Image Caption Machine.
	"""

    def __init__(self):
        """Start the API services.
        """

        self.welcome_service = rospy.ServiceProxy(
            "icm/machine/welcome", MachineService)
        self.navigate_service = rospy.ServiceProxy(
            "icm/machine/navigate", MachineService)
        self.learn_service = rospy.ServiceProxy(
            "icm/machine/learn", MachineService)
        self.where_service = rospy.ServiceProxy(
            "icm/machine/where", MachineService)
        self.caption_service = rospy.ServiceProxy(
            "icm/machine/caption", MachineService)
        self.recite_service = rospy.ServiceProxy(
            "icm/machine/recite", MachineService)
        self.help_service = rospy.ServiceProxy(
            "icm/machine/help", MachineService)
        self.stop_service = rospy.ServiceProxy(
            "icm/machine/stop", MachineService)


	def welcome(self):
        """Provide useful info when the app starts.
        """

        return self.welcome_service("").output


    def navigate(self, name):
        """The user asked to navigate to a location.
        Example:
            Navigate to the "Office".
        """

        return self.navigate_service(name).output


    def learn(self, name):
        """Learn the robots position as the given name.
        Example:
            This location is my "Office".
        """

        return self.learn_service(name).output


    def where(self):
        """Check the robots map find the closest place.
        Example:
            Where place are you.
        """

        return self.where_service("")


    def caption(self):
        """Read an incoming image and process a caption asynchronously.
        Example:
            Look around you.
        """

        return self.caption_service("").output


    def recite(self):
        """Speak out the latest image caption.
        Example:
            Tell what you saw.
        """

        return self.recite_service("").output


    def help(self):
        """The user has asked for help using the application.
        Example:
            Help.
        """

        return self.help_service("").output


    def stop(self):
        """The user has asked to exit the application.
        Example:
            Stop.
        """

        return self.stop_service("").output
