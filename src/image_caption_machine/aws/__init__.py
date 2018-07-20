"""Author: Brandon Trabucco
Utility functions to interact with AWS.
"""


from image_caption_machine.aws.aws_client import AWSClient


_aws_client = AWSClient()


def get_captions_with_aws(image):
    """Run the AWS client to obtain captions.
    Returns:
       List of [[str(caption), float(probability)], ...]
    """

    return _aws_client.post(image)
