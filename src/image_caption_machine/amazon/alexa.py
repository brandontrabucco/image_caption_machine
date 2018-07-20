"""Author: Brandon Trabucco
Alexa handler running inside ROS.
"""


import rospy
from flask_ask import statement, question, session


from image_caption_machine import ImageCaptionMachine


class Alexa(object):
    """Server class for alexa, where fields are event callbacks.
    """

    def __init__(self):
        """Build the internal caption machine.
        """

        rospy.init_node("alexa_server")
        self.machine = ImageCaptionMachine()


    def app_init(self):
        """The app has just started and is running.
        """

        return question(self.machine.welcome())


    def handle_navigate(self, message):
        """The user has just asked us to navigate to a location.
        Example:
            Navigate to the "Office".
        """
    
        return question(self.machine.navigate(message))


    def handle_learn(self, message):
        """Learn the robots position as the given name.
        Example:
            This location is my "Office".
        """

        return question(self.machine.learn(message))


    def handle_where(self):
        """Check the robots map find the closest place.
        Example:
            Where place are you.
        """

        return question(self.machine.where())


    def handle_caption(self):
        """Read an incoming image and process a caption asynchronously.
        Example:
            Look around you.
        """

        return question("").reprompt(self.machine.caption())


    def handle_recite(self):
        """Speak out the latest image caption.
        Example:
            Tell what you saw.
        """

        return question("").reprompt(self.machine.recite())


    def handle_stop(self):
        """The user has asked to exit the application.
        Example:
            Stop.
        """

        return statement(self.machine.stop())


    def handle_help(self):
        """The user has asked for help using the application.
        Example:
            Help.
        """

        return question(self.machine.help()).reprompt(" ")

