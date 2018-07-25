"""Author: Brandon Trabucco.
Adapted from Matthew Wilson/shmoo_echo/nodes/nav_patch.py
"""

import tf2_ros
import tf2_geometry_msgs
import rospy
from std_msgs.msg import Bool
from flat_ball.msg import TrajWithBackup
from geometry_msgs.msg import PoseStamped


from image_caption_machine.utils import get_pose_stamped
from image_caption_machine.world.place import Place
from image_caption_machine.navigator.abstract import Abstract


class Helper(Abstract):
    """Utility class to manage navigation with DFNav.
    """

    def __init__(self):
        """Initialize the publishers and subscribers.
        """

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer)

        self.goal_publisher = rospy.Publisher(
            "/goal", PoseStamped, queue_size=2)
        self.goal_subscriber = rospy.Subscriber(
            "/goal", PoseStamped, 
            self.handle_goal, queue_size=2)

        self.abort_publisher = rospy.Publisher(
            "/df/replan/abort", Bool, queue_size=1)
        self.abort_subscriber = rospy.Subscriber(
            "/df/replan/abort", Bool, 
            self.handle_abort, queue_size=1)

        self.replan_time = None
        self.replan_sub = rospy.Subscriber(
            '/df/traj', TrajWithBackup, 
            self.handle_replan, queue_size=10)

        self.is_finished = True
        self.on_complete = (lambda: None)


    def handle_goal(self, goal_pose):
        """Calculate if we are close enough to the goal.
        """

        transform = self.tf_buffer.lookup_transform(
            "map", "body", rospy.Time(0))
        goal_pose = tf2_geometry_msgs.do_transform_pose(
            goal_pose, transform)

        def update():
            """Checks whether the navigation is finished.
            """
	    
	    replan_expired = (self.replan_time is not None and (
	        rospy.Time.now() - self.replan_time).to_sec() > 4.0)
	    curr_pose = get_pose_stamped(self.tf_buffer)
	    near_goal = (curr_pose is not None and (
	        Place(pose_stamped=curr_pose).to(Place(pose_stamped=goal_pose)) < 0.1))
	    self.is_finished = (replan_expired or near_goal)

        r = rospy.Rate(10)
        self.is_finished = False
        while not rospy.is_shutdown() and not self.is_finished:
        	update()
        	r.sleep()
        self.replan_time = None
        self.on_complete()


    def handle_abort(self, abort):
        """Should the navigation be aborted.
        """

        if abort.data:
            self.is_finished = True


    def handle_replan(self, plan):
        """Track the replan time of dfnav to determine completion.
        """

        self.replan_time = rospy.Time.now()


    def go(self, goal_pose):
        """Start a navigation command
        """

        if self.is_finished:
            transform = self.tf_buffer.lookup_transform(
                "body", "map", rospy.Time(0))
            goal_pose = tf2_geometry_msgs.do_transform_pose(
                goal_pose, transform)
            self.goal_publisher.publish(goal_pose)


    def callback(self, fn):
        """Callback function runs after goal is reached or aborted.
        """

        def _on_complete():
            """Runs after navigation finishes.
            """

            fn()
            self.on_complete = (lambda: None)
            self.is_finished = True

        self.on_complete = _on_complete


    def finished(self):
    	"""Check if the navigation is finished.
    	"""

    	return self.is_finished


    def abort(self):
        """Quickly halt the navigation process.
        """

        self.abort_publisher.publish(Bool(True))
