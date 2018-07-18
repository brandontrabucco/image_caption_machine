#!/usr/bin/env python


"""Author: Brandon Trabucco
Alexa Flask Server running the image caption machine.
"""


from flask import Flask, render_template
from flask_ask import Ask, statement, question, session


app = Flask(__name__)
ask = Ask(app, "/")


import image_caption_machine
machine = image_caption_machine.get_machine()


@ask.launch
def app_init():
    """The app has just started and is running.
    """

    return question(machine())


@ask.intent(
    "NavigateIntent",
    mapping={"message": "Query"})
def handle_navigate(message):
    """The user has just asked us to navigate to a location.
    Example:
        Navigate to the "Office".
    """
    
    return question(machine.navigate(message))


@ask.intent(
    "LearnIntent",
    mapping={"message": "Query"})
def handle_learn(message):
    """Learn the robots position as the given name.
    Example:
        This location is my "Office".
    """

    return question(machine.learn(message))


@ask.intent(
    "WhereIntent")
def handle_learn(message):
    """Check the robots map find the closest place.
    Example:
        Where place are you.
    """

    return question(machine.where())


@ask.intent(
    "CaptionIntent")
def handle_caption():
    """Read an incoming image and process a caption asynchronously.
    Example:
        Look around you.
    """

    return question("").reprompt(machine.caption())


@ask.intent(
    "ReciteIntent")
def handle_recite():
    """Speak out the latest image caption.
    Example:
        Tell what you saw.
    """

    return question("").reprompt(machine.recite())


@ask.intent(
    "StopIntent")
def handle_stop():
    """The user has asked to exit the application.
    Example:
        Stop.
    """

    return statement(machine.stop())


@ask.intent(
    "HelpIntent")
def handle_help():
    """The user has asked for help using the application.
    Example:
        Help.
    """

    return question(machine.help()).reprompt(" ")


if __name__ == '__main__':
    app.run()

