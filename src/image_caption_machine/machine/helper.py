"""Author: Brandon Trabucco.
The engine class behind the image captioner.
"""


import tf2_ros
import rospy
import threading


from image_caption_machine.values import *
from image_caption_machine.utils import get_pose_stamped
from image_caption_machine.machine.abstract import Abstract
from image_caption_machine.world.place import Place
from image_caption_machine.world import Client as World
from image_caption_machine.captioner import Client as Captioner
from image_caption_machine.navigator import Client as Navigator


class Helper(Abstract):
    """Manages the core functionality of the app.
    """

    def __init__(self):
        """Initialize the app engine.
        """

        self.caption_text = "Unknown."
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer)
        
        self.places = World()
        self.captioner = Captioner()
        self.navigator = Navigator()


    def welcome(self):
        """Provide useful info when the app starts.
        """

        return (WELCOME_STRING)


    def navigate(self, name):
        """The user asked to navigate to a location.
        Example:
            Navigate to the "Office".
        """

        destination = self.places.lookup(name)
        if not destination:
            return UNK_STRING.format(
                name, self.places.length(), self.places.string())
        curr_pose = get_pose_stamped(self.tf_buffer)
        if not curr_pose:
            return LOST_STRING
        self.navigator.go(destination.pose_stamped)
        return NAV_STRING.format(
            name, Place(pose_stamped=curr_pose).to(destination))


    def learn(self, name):
        """Learn the robots position as the given name.
        Example:
            This location is my "Office".
        """

        destination = self.places.lookup(name)
        if destination:
            return KNOWN_STRING.format(name)
        curr_pose = get_pose_stamped(self.tf_buffer)
        if not curr_pose:
            return LOST_STRING
        self.places.append(Place(name=name, pose_stamped=curr_pose))
        return LEARN_STRING.format(name)


    def where(self):
        """Check the robots map find the closest place.
        Example:
            Where place are you.
        """

        curr_pose = get_pose_stamped(self.tf_buffer)
        if not curr_pose:
            return LOST_STRING
        p, distance = self.places.closest(Place(pose_stamped=curr_pose))
        return WHERE_STRING.format(p.name, distance)


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
                    self.caption_text = STILL_THINK_STRING
                    self.caption_text = self.captioner.caption()
                except Exception, e:
                    self.caption_text = ERROR_STRING
                    
            threading.Thread(target=do_work).start()

        except Exception, e:
            rospy.logerr(str(e))
            return ERROR_STRING

        return THINK_STRING


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

        return HELP_STRING.format(
            self.places.length(), self.places.string())


    def stop(self):
        """The user has asked to exit the application.
        Example:
            Stop.
        """

        return STOP_STRING

