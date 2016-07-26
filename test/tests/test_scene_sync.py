#!/usr/bin/env python

import unittest
import numpy as np
import rospy
import rostest
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotState

"""
Verifies https://github.com/ros-planning/moveit_ros/issues/442
"""

class TestSceneSync(unittest.TestCase):
    PLANNING_GROUP = "manipulator"
    OFFSET         = 0.2    # delta between generated positions
    TOL            = 1e-6   # tolerance for comparing positions

    @classmethod
    def setUpClass(self):
        self.group = MoveGroupCommander(self.PLANNING_GROUP)
        self.joint_names = self.group.get_active_joints()

    @classmethod
    def tearDownClass(self):
        pass

    @classmethod
    def generate_moves(self):
        curPos = self.group.get_current_joint_values()

        moves = []
        moves.append( [x + 1 * self.OFFSET for x in curPos] )
        moves.append( [x + 2 * self.OFFSET for x in curPos] )
        moves.append( curPos )
        return moves

    def sort_joints(self, pos, joints, ref):
        result = []
        for jnt in ref:
            result.append(pos[joints.index(jnt)])
        return result

    def validate_plan(self, current, plan, goal):
        t0 = self.sort_joints(plan.joint_trajectory.points[0].positions,
                              plan.joint_trajectory.joint_names,
                              self.joint_names)
        delta = np.array(t0) - np.array(current)
        self.assertTrue(np.all(np.abs(delta) < self.TOL), "Mismatch on start point")

        tN = self.sort_joints(plan.joint_trajectory.points[-1].positions,
                              plan.joint_trajectory.joint_names,
                              self.joint_names)
        delta = np.array(tN) - np.array(goal)
        self.assertTrue(np.all(np.abs(delta) < self.group.get_goal_joint_tolerance()),
                        "Mismatch on end point")

    def test(self):
        moves = self.generate_moves()

        for i in range(len(moves)):
            rospy.loginfo("Planning move to pos %i" % (i+1))
            self.group.set_joint_value_target(moves[i])

            current = self.group.get_current_joint_values()
            plan = self.group.plan()
            self.validate_plan(current, plan, moves[i])
            rospy.loginfo("Starting move to pos %i" % (i+1))
            self.group.execute(plan)

if __name__ == '__main__':
    PKGNAME = 'moveit_ros_tests'
    NODENAME = 'moveit_test_scene_sync'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, TestSceneSync)
