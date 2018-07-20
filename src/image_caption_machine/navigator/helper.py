"""Author: Brandon Trabucco.
Adapted from Matthew Wilson/shmoo_echo/nodes/nav_patch.py
"""


import tf
import rospy
from std_msgs.msg import Bool
from flat_ball.msg import TrajWithBackup
from geometry_msgs.msg import PoseStamped


from image_caption_machine.utils import get_pose
from image_caption_machine.world.place import Place
from image_caption_machine.navigator.abstract import Abstract


class Helper(Abstract):
    """Utility class to manage navigation with DFNav.
    """

    def __init__(self):
        """Initialize the publishers and subscribers.
        """

        self.tf_listener = tf.TransformListener()

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
        self.proximity = rospy.get_param("proximity_threshold")


    def handle_goal(self, goal_pose):
        """Calculate if we are close enough to the goal.
        """

        def update():
            """Checks whether the navigation is finished.
            """
	        
	    replan_expired = (self.replan_time is not None and (
	        rospy.Time.now() - self.replan_time).to_sec() > 4.0)
	    curr_pose = get_pose(self.tf_listener)
	    near_goal = (curr_pose is not None and (
	        Place(pose=curr_pose).to(Place(pose_stamped=goal_pose)) < self.proximity))
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
            self.goal_publisher.publish(goal_pose)


    def callback(self, fn):
        """Callback function runs after goal is reached or aborted.
        """

        def _on_complete():
            """Runs after navigation finishes.
            """

            fn()
            self.on_complete = (lambda: None)

        self.on_complete = _on_complete


    def finished(self):
    	"""Check if the navigation is finished.
    	"""

    	return self.is_finished


    def abort(self):
        """Quickly halt the navigation process.
        """

        self.abort_publisher.publish(Bool(True))
