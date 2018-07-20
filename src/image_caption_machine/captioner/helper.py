"""Author: Brandon Trabucco
A utility for requesting captions.
"""


from image_caption_machine.aws import get_captions_with_aws


class Helper(object):
    """Class for requsting image captions.
    """

    def caption(self, image):
        """Returns captions from an image.
        Args:
            image: numpy array of image pixels
        Returns:
            captions: str(...)
        """

        return get_captions_with_aws(image)
