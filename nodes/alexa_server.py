#!/usr/bin/env python


"""Author: Brandon Trabucco
Alexa Flask Server running the image caption machine.
"""


import rospy
from flask import Flask
from flask_ask import Ask


from image_caption_machine.amazon.alexa import Alexa


app = Flask(__name__)
ask = Ask(app, "/")
alexa = Alexa()


@ask.launch
def app_init():
    """The app has just started and is running.
    """

    return alexa.app_init()


@ask.intent(
    "NavigateIntent",
    mapping={"message": "Query"})
def handle_navigate(message):
    """The user has just asked us to navigate to a location.
    Example:
        Navigate to the "Office".
    """
    
    return alexa.handle_navigate(message)


@ask.intent(
    "LearnIntent",
    mapping={"message": "Query"})
def handle_learn(message):
    """Learn the robots position as the given name.
    Example:
        This location is my "Office".
    """

    return alexa.handle_learn(message)


@ask.intent(
    "WhereIntent")
def handle_where():
    """Check the robots map find the closest place.
    Example:
        Where place are you.
    """

    return alexa.handle_where()


@ask.intent(
    "CaptionIntent")
def handle_caption():
    """Read an incoming image and process a caption asynchronously.
    Example:
        Look around you.
    """

    return alexa.handle_caption()


@ask.intent(
    "ReciteIntent")
def handle_recite():
    """Speak out the latest image caption.
    Example:
        Tell what you saw.
    """

    return alexa.handle_recite()


@ask.intent(
    "StopIntent")
def handle_stop():
    """The user has asked to exit the application.
    Example:
        Stop.
    """

    return alexa.handle_stop()


@ask.intent(
    "HelpIntent")
def handle_help():
    """The user has asked for help using the application.
    Example:
        Help.
    """

    return alexa.handle_help()


if __name__ == '__main__':
    app.run()

