"""Author: Brandon Trabucco
A utility for requesting captions.
"""


from image_caption_machine.amazon import get_captions_with_aws
from image_caption_machine.captioner.abstract import Abstract


class Helper(Abstract):
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
