#!/usr/bin/env python3
"""
Unit test file to ensure this bot ain't crashlanding on ya
"""
from arm_move.srv import Step

import rospy
import unittest
from std_srvs.srv import Empty, EmptyResponse
from moveit_msgs.msg import MoveItErrorCodes
import rostest
from geometry_msgs.msg import Pose

import sys
import copy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Huge thanks to Mr. Chamzas for letting me talk my way through it all
# and for always helping when needed

class CrashLanding(unittest.TestCase):
    def __init__(self, *args):
        # I first learned how to use *args on machine learning
        # and I have never been more disappointed
        # super(CrashLanding, self).__init__(*args)
        # But this class introduced me to super which is...well, super
            # Ms. Smith in particular

    def test_crash(self):
        '''
        Tests to see whether mover returns the correct error @ collision:
            arguments: none
            returns: none (but kind of whether the test was SUCCESS or FAIL)
            exceptions: none
        '''
        rospy.init_node("test")
        rospy.sleep(1)
        # rospy.wait_for_service('/px100/reset')
        srvs = rospy.ServiceProxy('/px100/step', Step)
        srvr = rospy.ServiceProxy('/px100/reset', Empty)
        err = srvr.call()

        # points generously donated by Mr. Chamzas
        pt = Pose()
        pt.position.x = 0.0
        pt.position.y = 0.2
        pt.position.z = -0.1
        pt.orientation.x = 0
        pt.orientation.y = 0
        pt.orientation.z = 0.7
        pt.orientation.w = 0.7

        stp = Step()
        # stp.pose = pt
        stp.pose = copy.deepcopy(pt)
        stp.gripd = "false"

        # rospy.wait_for_service('/px100/step')
        # srvs = rospy.ServiceProxy('/px100/step', Step)
        # err = srvs.call(stp.pose, stp.gripd)
        err = srvs.call(stp)

        result = False

        if(err.code.val==-1):
            result = True
        self.assertEquals(result, True)

if __name__ == "__main__":
    rostest.rosrun('arm_move', "crashlanding.py", CrashLanding)