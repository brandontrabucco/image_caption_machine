"""Author: Brandon Trabucco.
Utility class for running ROS node world helper.
"""


import rospy


from image_caption_machine.world.place import Place
from image_caption_machine.world.helper import Helper
from image_caption_machine.world.abstract import Abstract
from image_caption_machine.srv import WorldAppend, WorldAppendResponse
from image_caption_machine.srv import WorldLookup, WorldLookupResponse
from image_caption_machine.srv import WorldString, WorldStringResponse
from image_caption_machine.srv import WorldLength, WorldLengthResponse
from image_caption_machine.srv import WorldClosest, WorldClosestResponse


class Server(Abstract):
    """Utility class to serve database of locations
    """

    def __init__(self):
        """Locate the database JSON file and prepopulate.
        """

        rospy.init_node("world_server")
        self.helper = Helper()
        self.append_service = rospy.Service(
            "icm/world/append", WorldAppend, self.append)
        self.lookup_service = rospy.Service(
            "icm/world/lookup", WorldLookup, self.lookup)
        self.string_service = rospy.Service(
            "icm/world/string", WorldString, self.string)
        self.length_service = rospy.Service(
            "icm/world/length", WorldLength, self.length)
        self.closest_service = rospy.Service(
            "icm/world/closest", WorldClosest, self.closest)
        rospy.spin()


    def append(self, m):
        """Add a new place to memory.
        Args:
            m: WorldAppend message.
        """

        self.helper.append(Place(msg=m.origin))
        return WorldAppendResponse()


    def lookup(self, m):
        """If this place exists, return it.
        Args:
            m: WorldLookup message.
        """

        place = self.helper.lookup(m.name)
        if place is not None:
            return WorldLookupResponse(place.msg, True)
        return WorldLookupResponse(Place(name="None", x=-1, y=-1).msg, False)


    def string(self, m):
        """Return a list of place names
        """

        return WorldStringResponse(
            self.helper.string())


    def length(self, m):
        """Return the number of places.
        """

        return WorldLengthResponse(
            self.helper.length())


    def closest(self, m):
        """Return the closest place and distance to q.
        Args:
            m: WorldClosest message.
        """
        place, distance = self.helper.closest(Place(msg=m.origin))
        return WorldClosestResponse(place.msg, distance)


