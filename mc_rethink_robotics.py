#!/usr/bin/env python
# -*- coding: utf-8 -*-

import mc_control
import mc_rbdyn
import mc_rtc

import rospy
import rospkg

import intera_interface
from intera_interface import CHECK_VERSION

import os
import subprocess
import sys

from math import floor

def make_sawyer_urdf(sawyer_desc):
    xacro_in = sawyer_desc + '/urdf/sawyer.urdf.xacro'
    urdf_out = sawyer_desc + '/urdf/sawyer.urdf'
    if not os.path.exists(xacro_in):
        print "No xacro file in {}: something is wrong".format(sawyer_desc)
        sys.exit(1)
    xacro_tm = os.path.getmtime(xacro_in)
    if os.path.exists(urdf_out):
        urdf_tm = os.path.getmtime(urdf_out)
    else:
        urdf_tm = 0
    if xacro_tm > urdf_tm:
        print "Generating sawyer urdf with xacro"
        err = subprocess.call('xacro --inorder -o {} {}'.format(urdf_out, xacro_in).split())
        if err:
            print "Failed to generated URDF, check above for errors"
            sys.exit(1)

if __name__ == "__main__":
    rospy.init_node("mc_rethink_robotics")

    try:
        sawyer_desc = rospkg.RosPack().get_path('sawyer_description')
    except rospkg.ResourceNotFound:
        print "No sawyer_description package available, aborting..."
        sys.exit(1)
    make_sawyer_urdf(sawyer_desc)
    rm = mc_rbdyn.get_robot_module("env", sawyer_desc, 'sawyer')

    gc = mc_control.MCGlobalController("", rm)
    dt = gc.timestep()
    robot = gc.robot()

    #FIXME Make default control mode a launch option
    torque_control = False
    def enable_torque_control():
        global torque_control
        torque_control = True
        gc.controller().read_msg("torque 1")
    def disable_torque_control():
        global torque_control
        torque_control = False
        gc.controller().read_msg("torque 0")
    def toggle_torque_control():
        if torque_control:
            print "Enabling position control"
            disable_torque_control()
        else:
            print "Enabling torque control"
            enable_torque_control()
    gc.controller().gui().addElement([], mc_rtc.gui.Checkbox("Torque control", lambda: torque_control, toggle_torque_control))

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def on_shutdown():
        if not init_state:
            rs.disable()
    rospy.on_shutdown(on_shutdown)

    print "Enabling Sawyer..."
    rs.enable()
    print "Sawyer enabled"

    limb = intera_interface.Limb()
    limb.set_joint_position_speed(0.5)

    rjo = gc.ref_joint_order()
    arm_rjo = limb.joint_names()
    rji = [robot.jointIndexByName(j) for j in arm_rjo]
    print "rjo",rjo
    print "arm_rjo",arm_rjo

    def get_encoders():
        jstate = limb.joint_angles()
        return [jstate.get(j, 0) for j in rjo]

    def get_encoders_alpha():
        jstate = limb.joint_velocities()
        return [jstate.get(j, 0) for j in rjo]

    def get_torques():
        jstate = limb.joint_efforts()
        return [jstate.get(j, 0) for j in rjo]

    def update_sensors():
        gc.setEncoderValues(get_encoders())
        gc.setEncoderVelocities(get_encoders_alpha())
        gc.setJointTorques(get_torques())

    def position_target():
        return {j: robot.mbc.q[ji][0] for j,ji in zip(arm_rjo,rji)}

    def torque_target():
        return {j: robot.mbc.jointTorque[ji][0] for j,ji in zip(arm_rjo,rji)}

    def send_result():
        if torque_control:
            limb.set_joint_torques(torque_target())
        else:
            limb.set_joint_positions(position_target())

    gc.init(get_encoders())

    go_to_neutral = False
    def move_to_neutral():
        global go_to_neutral
        gc.running = False
        go_to_neutral = True
    gc.controller().gui().addElement([], mc_rtc.gui.Button("Go to neutral", move_to_neutral))

    rate = floor(1/dt)
    rt = rospy.Rate(rate)
    gc.running = True
    print "Running controller at {}Hz".format(rate)
    while not rospy.is_shutdown():
        update_sensors()
        if gc.run():
            send_result()
        rt.sleep()
        if go_to_neutral:
            limb.move_to_neutral(timeout = 5)
            gc.init(get_encoders())
            gc.running = True
            go_to_neutral = False
