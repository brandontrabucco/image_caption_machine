"""Author: Brandon Trabucco.
Utility class for connecting to World server.
"""


import rospy


from image_caption_machine.msg import WorldPlace
from image_caption_machine.srv import WorldAppend
from image_caption_machine.srv import WorldLookup
from image_caption_machine.srv import WorldString
from image_caption_machine.srv import WorldLength
from image_caption_machine.srv import WorldClosest
from image_caption_machine.world.place import Place
from image_caption_machine.world.helper import Helper
from image_caption_machine.world.abstract import Abstract


class Client(Abstract):
    """Utility class to serve database of locations
    """

    def __init__(self):
        """Start connections to the world server.
        """

        rospy.wait_for_service('icm/world/append', timeout=10.0)
        self.append_service = rospy.ServiceProxy(
            "icm/world/append", WorldAppend)
        rospy.wait_for_service('icm/world/lookup', timeout=10.0)
        self.lookup_service = rospy.ServiceProxy(
            "icm/world/lookup", WorldLookup)
        rospy.wait_for_service('icm/world/string', timeout=10.0)
        self.string_service = rospy.ServiceProxy(
            "icm/world/string", WorldString)
        rospy.wait_for_service('icm/world/length', timeout=10.0)
        self.length_service = rospy.ServiceProxy(
            "icm/world/length", WorldLength)
        rospy.wait_for_service('icm/world/closest', timeout=10.0)
        self.closest_service = rospy.ServiceProxy(
            "icm/world/closest", WorldClosest)


    def append(self, p):
        """Add a new place to memory.
        Args:
            p: Place object.
        """

        self.append_service(p.msg)


    def lookup(self, x):
        """If this place exists, return it.
        Args:
            x: str
        """

        result = self.lookup_service(x)
        return (Place(msg=result.match) if result.found else None)


    def string(self):
        """Return a list of place names
        """

        return self.string_service().names


    def length(self):
        """Return the number of places.
        """

        return self.length_service().length


    def closest(self, q):
        """Return the closest place and distance to q.
        Args:
            q: Place object.
        """
        
        result = self.closest_service(q.msg)
        return Place(msg=result.match), result.distance

