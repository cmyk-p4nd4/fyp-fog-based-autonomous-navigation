#!/usr/bin/env python2.7
import roslaunch
import rospy
import os
import sys

path = os.path.join(os.getcwd(), 'maps/map_out.yaml')

pkg = 'map_server'
prog = 'map_server'
rospy.init_node('trigger', anonymous=True)
node = roslaunch.core.Node(pkg,prog,args=path)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
rospy.sleep(5)
process.stop()