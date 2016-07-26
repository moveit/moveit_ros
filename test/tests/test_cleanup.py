#!/usr/bin/env python

import unittest
import subprocess
import os
import rostest
import rospkg

PKGNAME = 'moveit_ros_tests'
NODENAME = 'moveit_test_cleanup'

# As issue #592 is related to a crash during program exit,
# we cannot perform a standard unit test.
# We have to check the return value of the called program.
# As rostest doesn't do this automatically, we do it ourselves
# and call the actual test program here.
class RunTest(unittest.TestCase):
    def test(self):
        rospack = rospkg.RosPack()
        cmd = os.path.join(rospack.get_path(PKGNAME), "movegroup_interface.py")
        self.assertTrue(subprocess.call([cmd]) == 0)

if __name__ == '__main__':
    rostest.rosrun(PKGNAME, NODENAME, RunTest)
