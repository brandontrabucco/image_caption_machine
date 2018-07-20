"""Author: Brandon Trabucco
String values for use in the caption machine.
"""


WELCOME_STRING = """

I'm ready to caption images for you.
Please tell me a command, or say help.

"""


HELP_STRING = """

Ask me to navigate,
learn a place,
tell where I am,
look around,
or tell what I see

I currently know {0} places.
These are {1}.

"""


UNK_STRING = """

I'm sorry, I do not recognize the {0}.
I currently know {1} places.
These are {2}.

"""


LOST_STRING = """

I'm sorry, I cannot determine my position.

"""


WHERE_STRING = """

I am closest to the {0},
which is about {1:.2f} units away.

"""


NAV_STRING = """

I am navigating to the {0},
which is about {1:.2f} units away.

"""


STOP_STRING = """

Thanks for talking with me, have a nice day.

"""


ERROR_STRING = """

There was an error with your request.

"""


THINK_STRING = """

Okay, I am thinking of a caption.

"""


STILL_THINK_STRING = """

Still thinking of a caption.

"""


KNOWN_STRING = """

I already know where the {0} is located.

"""


LEARN_STRING = """

I have added {0} to my memory.

"""
