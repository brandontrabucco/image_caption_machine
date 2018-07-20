"""Author: Brandon Trabucco
An interface for building an image captioner.
"""


class Abstract(object):
    """Class for requsting image captions.
    """

    def caption(self, image):
        """Returns captions from an image.
        Args:
            image: numpy array of image pixels
        Returns:
            captions: str(...)
        """

        pass
