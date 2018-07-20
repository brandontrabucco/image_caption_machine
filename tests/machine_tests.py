#!/usr/bin/env python


"""Author: Brandon Trabucco
Tests the major functionality of the Image Caption Machine.
"""


import tf
import rospy


from image_caption_machine import ImageCaptionMachine as get_machine


if __name__ == "__main__":

    """Create the node.
    """
    rospy.init_node("machine_tests")
    m = get_machine(tf.TransformListener())
    rospy.loginfo("Starting tests.")


    """Test the major fields.
    """
    rospy.loginfo("Places: " + str(m.places))
    rospy.loginfo("__call__: " + m())
    rospy.loginfo("Help: " + m.help())
    rospy.loginfo("Stop: " + m.stop())
    rospy.loginfo("Navigate: " + m.navigate("default"))
    rospy.loginfo("Learn: " + m.learn("default"))
    rospy.loginfo("Where: " + m.where())
    rospy.loginfo("Caption: " + m.caption())
    rospy.loginfo("Recite: " + m.recite())


    """Finished, and hopefully a success.
    """
    rospy.loginfo("All test cases finished.")
