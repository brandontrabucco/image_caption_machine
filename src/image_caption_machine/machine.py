"""Author: Brandon Trabucco.
The engine class behind the image captioner.
"""


import tf
import rospy
import threading
import image_caption_machine
import image_caption_machine.utils
import image_caption_machine.srv


from image_caption_machine.place import Place


WELCOME_STRING = """

I'm ready to caption images for you.
Please tell me a command, or say help.

"""


HELP_STRING = """

Ask me to navigate, 
learn a place, 
tell where I am,
look around, 
or tell what I see

I currently know {0} places.
These are {1}.

"""


UNK_STRING = """

I'm sorry, I do not recognize the {0}.
I currently know {1} places.
These are {2}.

"""


LOST_STRING = """

I'm sorry, I cannot determine my position.

"""


WHERE_STRING = """

I am closest to the {0}, 
which is about {1} units away.

"""


class ImageCaptionMachine(object):
    """Manages the core functionality of the app.
    Fields:
        .places
        .__call__()
        .navigate(name)
        .learn(name)
        .where()
        .caption()
        .recite()
        .help()
        .exit()
    """

    def __init__(self):
        """Initialize the app engine.
        """
        
        rospy.init_node("image_caption_machine")
        self.caption_text = "Unknown."


    @property
    def places(self):
        """Returns the places known globally.
        """

        return image_caption_machine.get_places()


    def __call__(self):
        """Provide useful info when the app starts.
        """
        
        return (WELCOME_STRING)


    def navigate(self, name):
        """The user asked to navigate to a location.
        Example:
            Navigate to the "Office".
        """
    
        destination = name in self.places
        if not destination:
            return UNK_STRING.format(
                name, len(self.places), str(self.places))
        return (
            "Your message was: " +
            name + 
            ". Sorry, but I cannot navigate yet.")


    def learn(self, name):
        """Learn the robots position as the given name.
        Example:
            This location is my "Office".
        """

        destination = name in self.places
        if destination:
            return ("I already know where the %s is located." % name)
        curr_odom = image_caption_machine.utils.get_odom()
        if not curr_odom:
            return LOST_STRING
        self.places.append(Place(name=name, odom=curr_odom))
        return ("I have added " + name + " to my memory.")


    def where(self):
        """Check the robots map find the closest place.
        Example:
            Where place are you.
        """

        curr_odom = image_caption_machine.utils.get_odom()
        if not curr_odom:
            return LOST_STRING
        name, distance = self.places.closest(Place(odom=curr_odom))
        return WHERE_STRING.format(name, str(distance))


    def caption(self):
        """Read an incoming image and process a caption asynchronously.
        Example:
            Look around you.
        """

        try:
            def do_work():
                """Compute the image caption in another thread.
                """

                try:
                    self.caption_text = ("Still thinking of a caption.")
                    rospy.wait_for_service("caption_client", timeout=10.0)
                    response = rospy.ServiceProxy(
                        "caption_client", 
                        image_caption_machine.srv.CaptionString)()
                    self.caption_text = response.caption_text

                except Exception, e:
                    self.caption_text = ("There was an error with the caption.")

            t = threading.Thread(target=do_work)
            t.start()

        except Exception, e:
            rospy.logerr(str(e))
            return ("There was an error with your request.")

        return ("Okay, I am thinking of a caption.")


    def recite(self):
        """Speak out the latest image caption.
        Example:
            Tell what you saw.
        """
    
        return self.caption_text
  

    def help(self):
        """The user has asked for help using the application.
        Example:
            Help.
        """

        return HELP_STRING.format(len(self.places), str(self.places))


    def stop(self):
        """The user has asked to exit the application.
        Example:
            Stop.
        """

        return ("Thanks for talking with me, have a nice day.")










