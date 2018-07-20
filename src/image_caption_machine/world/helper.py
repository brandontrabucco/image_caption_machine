"""Author: Brandon Trabucco.
Utility class for loading and managing locations in the robot's map.
"""


import rospy
import json
import math
import os.path


from image_caption_machine.world.place import Place
from image_caption_machine.world.abstract import Abstract


class Helper(Abstract):
    """Utility class to manage the database of known places.
    """

    def __init__(self):
        """Locate the database JSON file and prepopulate.
        """

        self.fname = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), "world.json")
        self._create()


    def _dump(self):
        """Serialize the list of places to the JSON file.
        """

        with open(self.fname, "w") as f:
            json.dump([x.json for x in self.content], f)


    def _create(self):
        """Populate an empty file if necessary.
        """

        if not os.path.isfile(self.fname):
            with open(self.fname, "w") as f:
                json.dump([Place().json], f)
        self._load()


    def _load(self):
        """Load the JSON database into memory.
        """

        if os.path.isfile(self.fname):
            with open(self.fname, "r") as f:
                self.content = [Place(json=x) for x in json.load(f)]


    def append(self, p):
        """Add a new place to memory.
        """

        self.content.append(p)
        self._dump()


    def lookup(self, x):
        """If this place exists, return it.
        """

        for y in self.content:
            if y.name == x:
                return y
        return None


    def string(self):
        """Return a list of place names
        """

        return ", ".join([x.name for x in self.content])


    def length(self):
        """Return the number of places.
        """

        return len(self.content)


    def closest(self, q):
        """Return the closest place and distance to q.
        """

        best_place = None
        best_distance = float("inf")
        for p in self.content:
            current_distance = q.to(p)
            if current_distance < best_distance:
                best_place = p
                best_distance = current_distance
        return best_place, best_distance


