"""Author: Brandon Trabucco.
Expose a public API for the AWSClient.
"""


from image_caption_machine.aws.aws_client import AWSClient
from image_caption_machine.world_helper import WorldHelper
from image_caption_machine.machine import ImageCaptionMachine


_aws_client = AWSClient()


def get_captions(image):
    """Run the AWS client to obtain captions.
    Returns:
       List of [[str(caption), float(probability)], ...]
    """

    return _aws_client.post(image)


def get_places():
    """Get the list of known places.
    Returns:
        WorldHelper() object.
    """

    return WorldHelper()


def get_machine():
    """Get a new running image caption machine.
    returns:
        ImageCaptionMachine() object
    """

    return ImageCaptionMachine()
