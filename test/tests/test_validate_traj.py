#!/usr/bin/env python

import unittest
import numpy as np
import rospy
import rostest
from moveit_commander import MoveGroupCommander

class TestTrajectoryValidation(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        self.group = MoveGroupCommander(self.PLANNING_GROUP)

    @classmethod
    def tearDownClass(self):
        pass

    def test(self):
        current = np.asarray(self.group.get_current_joint_values())

        target1 = current + 0.2
        self.group.set_joint_value_target(target1)
        plan1 = self.group.plan()

        target2 = current + 0.4
        self.group.set_joint_value_target(target2)
        plan2 = self.group.plan()

        # first plan should execute
        self.assertTrue(self.group.execute(plan1))
        
        # second plan should be invalid now (due to modified start point) and rejected
        self.assertFalse(self.group.execute(plan2))
        

if __name__ == '__main__':
    PKGNAME = 'moveit_ros_tests'
    NODENAME = 'moveit_test_validate_traj'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, TestTrajectoryValidation)
