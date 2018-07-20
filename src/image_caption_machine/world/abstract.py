"""Author: Brandon Trabucco.
Utility interface to generalize the API for the world.
"""


class Abstract(object):
    """Interface for manipulating Places with common API.
    """

    def append(self, p):
        """Add a new place to memory.
        """

        pass


    def lookup(self, x):
        """If this place exists, return it.
        """

        pass


    def string(self):
        """Return a list of place names
        """

        pass


    def length(self):
        """Return the number of places.
        """

        pass


    def closest(self, q):
        """Return the closest place and distance to q.
        """

        pass
    

