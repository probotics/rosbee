#!/usr/bin/env python
"""
    fake_pub.py
    
    Fake joint state publisher.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    Modified by Eric Dortmans (e.dortmans@fontys.nl)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""
import roslib; roslib.load_manifest("rosbee2_configs")
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("fake_pub")
p = rospy.Publisher('joint_states', JointState)

msg = JointState()
msg.name = ["base_l_wheel_joint", "base_r_wheel_joint", "base_f_wheel_joint", "base_b_wheel_joint"]
msg.position = [0.0 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.1)

