#!/usr/bin/env python3

import rospy
import math
import unittest
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TestWaypointPrecision(unittest.TestCase):

    def setUp(self):
        rospy.init_node('waypoint_test_node', anonymous=True)
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.assertTrue(self.client.wait_for_server(rospy.Duration(5.0)), "Action server not available")

        # Subscriber for final odometry
        self.final_odom = None
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        rospy.sleep(1.0)  # wait a bit for the subscriber to sync

    def _odom_callback(self, msg):
        self.final_odom = msg

    def _get_current_position_and_yaw(self):
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while not self.final_odom and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        self.assertIsNotNone(self.final_odom, "No odometry received")

        pos = self.final_odom.pose.pose.position
        ori = self.final_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        return pos, yaw

    def _send_goal(self, x, y):
        goal = WaypointActionGoal()
        goal.position = Point(x, y, 0.0)
        self.client.send_goal(goal)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(15)), "Timeout waiting for result")
        result = self.client.get_result()
        self.assertTrue(result.success, "Goal reported failure")

    def test_final_position_within_tolerance(self):
        """Check final [x, y] matches goal within a margin."""
        goal_x, goal_y = 0.5, 0.5
        self._send_goal(goal_x, goal_y)

        final_pos, _ = self._get_current_position_and_yaw()

        pos_tol = 0.1  # allowable position error
        self.assertAlmostEqual(final_pos.x, goal_x, delta=pos_tol, msg="X position out of tolerance")
        self.assertAlmostEqual(final_pos.y, goal_y, delta=pos_tol, msg="Y position out of tolerance")

    def test_final_yaw_within_tolerance(self):
        """Check final yaw (rotation) points toward the goal within a margin."""
        goal_x, goal_y = -0.5, -0.5
        self._send_goal(goal_x, goal_y)

        final_pos, final_yaw = self._get_current_position_and_yaw()

        # Expected yaw: angle from origin to goal
        expected_yaw = math.atan2(goal_y - final_pos.y, goal_x - final_pos.x)
        yaw_error = abs(math.atan2(math.sin(expected_yaw - final_yaw), math.cos(expected_yaw - final_yaw)))

        yaw_tol = 0.5  # rad ≈ ~6 degrees
        self.assertLessEqual(yaw_error, yaw_tol, "Yaw angle out of tolerance")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tortoisebot_waypoints', 'waypoint_test_node', TestWaypointPrecision)