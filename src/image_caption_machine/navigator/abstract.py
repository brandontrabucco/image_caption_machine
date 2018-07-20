"""Author: Brandon Trabucco
An interface for building an the navigator.
"""


class Abstract(object):
    """Class for requsting navigation commands.
    """

    def go(self, goal):
        """Start a navigation command
        """

        pass


    def callback(self, fn):
        """Run specified function after navigation.
        """

        pass


    def finished(self):
        """Check if the navigation is finished.
        """

        pass


    def abort(self):
        """Quickly halt the navigation process.
        """

        pass