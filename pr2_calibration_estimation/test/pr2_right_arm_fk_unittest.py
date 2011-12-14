#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import roslib; roslib.load_manifest('pr2_calibration_estimation')

import sys
import unittest
import rospy
import time

from calibration_estimation.full_chain import FullChainRobotParams
from calibration_estimation.urdf_params import UrdfParams
from sensor_msgs.msg import JointState

import yaml
from pr2_calibration_estimation.srv import FkTest
from numpy import *
import numpy

class LoadData(unittest.TestCase):
    def setUp(self):
        config = yaml.load(open(rospy.get_param('config_file')))
        self.robot_params = UrdfParams(rospy.get_param('robot_description'), config)

        rospy.wait_for_service('fk', 3.0)
        self._fk_ref = rospy.ServiceProxy('fk', FkTest)

    def loadCommands(self, param_name):
        command_str = rospy.get_param(param_name)
        cmds = [ [float(y) for y in x.split()] for x in command_str.strip().split('\n')]
        return cmds

    def getExpected(self, root, tip, cmd):
        resp = self._fk_ref(root, tip, cmd)
        T = matrix(zeros((4,4)), float)
        T[0:3, 0:3] = reshape( matrix(resp.rot, float), (3,3))
        T[3,3] = 1.0;
        T[0:3,3] = reshape( matrix(resp.pos, float), (3,1))
        return T

class TestPR2Fk(LoadData):

    def run_test(self, full_chain, root, tip, cmds):
        for cmd in cmds:
            print "On Command: %s" % cmd

            expected_T = self.getExpected(root, tip, cmd)
            chain_state = JointState(position=cmd)
            actual_T = full_chain.calc_block.fk(chain_state)

            print "Expected_T:"
            print expected_T
            print "Actual T:"
            print actual_T

            self.assertAlmostEqual(numpy.linalg.norm(expected_T-actual_T), 0.0, 6)


    def test_right_arm_fk(self):
        print ""
        full_chain = FullChainRobotParams('right_arm_chain','r_wrist_roll_link')
        full_chain.update_config(self.robot_params)
        cmds = self.loadCommands('r_arm_commands')
        self.run_test(full_chain, 'torso_lift_link', 'r_wrist_roll_link', cmds)


    def test_right_forearm_roll_fk(self):
        print ""
        full_chain = FullChainRobotParams('right_arm_chain','r_forearm_roll_link')
        full_chain.update_config(self.robot_params)
        cmds = self.loadCommands('r_arm_commands')
        self.run_test(full_chain, 'torso_lift_link', 'r_forearm_roll_link', cmds)


    def test_right_forearm_cam_fk(self):
        print ""
        full_chain = FullChainRobotParams('right_arm_chain','r_forearm_cam_frame')
        full_chain.update_config(self.robot_params)
        cmds = self.loadCommands('r_arm_commands')
        self.run_test(full_chain, 'torso_lift_link', 'r_forearm_cam_frame', cmds)


    def test_right_forearm_cam_optical_fk(self):
        print ""
        full_chain = FullChainRobotParams('right_arm_chain','r_forearm_cam_optical_frame')
        full_chain.update_config(self.robot_params)
        cmds = self.loadCommands('r_arm_commands')
        self.run_test(full_chain, 'torso_lift_link', 'r_forearm_cam_optical_frame', cmds)


if __name__ == '__main__':
    import rostest
    rospy.init_node("fk_test")
    rostest.unitrun('pr2_calibration_estimation', 'test_PR2Fk', TestPR2Fk)
