#!/usr/bin/env python


"""Author: Brandon Trabucco
Alexa Flask Server running the image caption machine.
"""


import rospy
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session


from image_caption_machine import ImageCaptionMachine as get_machine


app = Flask(__name__)
ask = Ask(app, "/")


class Alexa(object):
    """Server class for alexa, where fields are event callbacks.
    """

    def __init__(self):
        """Build the internal caption machine.
        """

        rospy.init_node("alexa_server")
        self.machine = get_machine(tf.TransformListener())


    @ask.launch
    def app_init(self):
        """The app has just started and is running.
        """

        return question(self.machine())


    @ask.intent(
        "NavigateIntent",
        mapping={"message": "Query"})
    def handle_navigate(self, message):
        """The user has just asked us to navigate to a location.
        Example:
            Navigate to the "Office".
        """
    
        return question(self.machine.navigate(message))


    @ask.intent(
        "LearnIntent",
        mapping={"message": "Query"})
    def handle_learn(self, message):
        """Learn the robots position as the given name.
        Example:
            This location is my "Office".
        """

        return question(self.machine.learn(message))


    @ask.intent(
        "WhereIntent")
    def handle_learn(self, message):
        """Check the robots map find the closest place.
        Example:
            Where place are you.
        """

        return question(self.machine.where())


    @ask.intent(
        "CaptionIntent")
    def handle_caption(self):
        """Read an incoming image and process a caption asynchronously.
        Example:
            Look around you.
        """

        return question("").reprompt(self.machine.caption())


    ask.intent(
        "ReciteIntent")
    def handle_recite(self):
        """Speak out the latest image caption.
        Example:
            Tell what you saw.
        """

        return question("").reprompt(self.machine.recite())


    @ask.intent(
        "StopIntent")
    def handle_stop(self):
        """The user has asked to exit the application.
        Example:
            Stop.
        """

        return statement(self.machine.stop())


    @ask.intent(
        "HelpIntent")
    def handle_help(self):
        """The user has asked for help using the application.
        Example:
            Help.
        """

        return question(self.machine.help()).reprompt(" ")


if __name__ == '__main__':
    alexa = Alexa()
    app.run()

