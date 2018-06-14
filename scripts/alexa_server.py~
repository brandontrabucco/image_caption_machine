#!/usr/bin/env python

from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
import rospy
from image_caption_machine.srv import CaptionString

app = Flask(__name__)
ask = Ask(app, "/")

@ask.launch

def app_init():
    return question(
        "Image Caption started. " +
        "What is your Command?")

@ask.intent(
    "QuestionIntent",
    mapping={"message": "Query"})

def handle_question(message):
    return question(
        "Your question was: " +
        message + 
        ". Sorry, but I cannot answer that yet.")

@ask.intent(
    "CaptionIntent")

def handle_caption():
    rospy.init_node("alexa_server")
    rospy.wait_for_service("caption_client")
    try:
        caption_fn = rospy.ServiceProxy(
            "caption_client", CaptionString)
        response = caption_fn()
        caption = response.caption_text
    except rospy.ServiceException:
        return question("There was an error with your request.")
    return question("This is what I see: " + caption)

@ask.intent("AMAZON.StopIntent")

def handle_stop():
    return statement("Have a nice day! Goodbye.")

if __name__ == '__main__':
    app.run()

