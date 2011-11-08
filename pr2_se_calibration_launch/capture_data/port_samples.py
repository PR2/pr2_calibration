#! /usr/bin/env python

# This script is used to convert PR2 Samples into PR2 Lite Samples
# Author Vijay Pradeep
# Modified 11/11 by Adam Leeper

import roslib; roslib.load_manifest('pr2_calibration_launch')
import rospy
import yaml
import sys
import os

input_dir = rospy.myargv()[1]
output_dir = rospy.myargv()[2]
sample_names = [x for x in os.listdir(input_dir) if ".yaml" in x]
sample_names.sort()
full_input_paths  = [input_dir  + "/" + x for x in sample_names]
full_output_paths = [output_dir + "/" + x for x in sample_names]

#TODO Add support for left OR right arm

for cur_input_path, cur_output_path in zip(full_input_paths, full_output_paths):
    print "On sample [%s]" % cur_input_path
    cur_config = yaml.load(open(cur_input_path))

    # Don't use any of the l_forearm samples
    if 'forearm_left_rect' not in [ x['cam_id'] for x in cur_config['camera_measurements']]:
        next_config = {}
        if 'forearm_right_rect' in [ x['cam_id'] for x in cur_config['camera_measurements']]:
            next_config['camera_measurements'] = [{'cam_id':'forearm_right_rect', 'config':'small_cb_4x5'}]
        else:
            next_config['camera_measurements'] = [{'cam_id':'kinect_rect', 'config':'small_cb_4x5'}, {'cam_id':'prosilica_rect', 'config':'small_cb_4x5'} ]
        next_config['joint_commands'] = [x for x in cur_config['joint_commands'] if x['controller'] in ['head_traj_controller', 'r_arm_controller']]
        next_config['joint_measurements'] = [x for x in cur_config['joint_measurements'] if x['chain_id'] in ['head_chain', 'right_arm_chain']]
        if 'tilt_laser' in [ x['laser_id'] for x in cur_config['laser_measurements']]:
            next_config['laser_measurements'] = [{'config': 'small_cb_4x5', 'laser_id': 'tilt_laser'}]
        else:
            next_config['laser_measurements'] = []
        next_config['sample_id'] = cur_config['sample_id']
        next_config['target'] = cur_config['target']

        # Tilt the head down by about 11 degrees to account for higher kinect position.
        # Yes, this is a total hack.
        for head_cmd in [x for x in next_config['joint_commands'] if x['controller'] == 'head_traj_controller']:
            for cur_segment in head_cmd['segments']:
                cur_segment['positions'][1] = cur_segment['positions'][1] + 0.2

        outfile = file(cur_output_path, 'w')
        yaml.dump(next_config, outfile)
        outfile.close()
    else:
        print "Skipping"


