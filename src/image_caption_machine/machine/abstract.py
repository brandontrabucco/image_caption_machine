"""Author: Brandon Trabucco
Define a public API for the Image Caption Machine.
"""


class Abstract(object):
    """Public API for an Image Caption Machine.
    """

    def welcome(self):
        """Provide useful info when the app starts.
        """

        pass


    def navigate(self, name):
        """The user asked to navigate to a location.
        Example:
            Navigate to the "Office".
        """

        pass


    def learn(self, name):
        """Learn the robots position as the given name.
        Example:
            This location is my "Office".
        """

        pass


    def where(self):
        """Check the robots map find the closest place.
        Example:
            Where place are you.
        """

        pass


    def caption(self):
        """Read an incoming image and process a caption asynchronously.
        Example:
            Look around you.
        """


    def recite(self):
        """Speak out the latest image caption.
        Example:
            Tell what you saw.
        """

        pass


    def help(self):
        """The user has asked for help using the application.
        Example:
            Help.
        """

        pass


    def stop(self):
        """The user has asked to exit the application.
        Example:
            Stop.
        """

        pass
