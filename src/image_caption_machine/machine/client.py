"""Author: Brandon Trabucco
Serve the Image Caption Machine on a ROS node.
"""


import rospy


from image_caption_machine.srv import Machine
from image_caption_machine.machine.abstract import Abstract


class Client(Abstract):
    """Utility class to serve Image Caption Machine.
    """

    def __init__(self):
        """Start the API services.
        """

        rospy.wait_for_service('icm/machine/welcome', timeout=10.0)
        self.welcome_service = rospy.ServiceProxy(
            "icm/machine/welcome", Machine)

        rospy.wait_for_service('icm/machine/navigate', timeout=10.0)
        self.navigate_service = rospy.ServiceProxy(
            "icm/machine/navigate", Machine)

        rospy.wait_for_service('icm/machine/learn', timeout=10.0)
        self.learn_service = rospy.ServiceProxy(
            "icm/machine/learn", Machine)

        rospy.wait_for_service('icm/machine/where', timeout=10.0)
        self.where_service = rospy.ServiceProxy(
            "icm/machine/where", Machine)

        rospy.wait_for_service('icm/machine/caption', timeout=10.0)
        self.caption_service = rospy.ServiceProxy(
            "icm/machine/caption", Machine)

        rospy.wait_for_service('icm/machine/recite', timeout=10.0)
        self.recite_service = rospy.ServiceProxy(
            "icm/machine/recite", Machine)

        rospy.wait_for_service('icm/machine/help', timeout=10.0)
        self.help_service = rospy.ServiceProxy(
            "icm/machine/help", Machine)

        rospy.wait_for_service('icm/machine/stop', timeout=10.0)
        self.stop_service = rospy.ServiceProxy(
            "icm/machine/stop", Machine)


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

        return self.where_service("").output


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
