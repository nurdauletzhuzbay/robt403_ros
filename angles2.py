#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

rospy.init_node('angles')
pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=5)
# pub1 = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=5)
# pub2 = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=5)
# pub3 = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=5)
# pub4 = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=5)
msg = [0, 0, 0, 0, 0]

cmd = 1.0
count = 0
count2 = 0
period = 50


rate = rospy.Rate(50)
while not False:
	M = 2
	count = count +1
	if count==10:
		count = 0
		count2 = count2+1
		if count2 == period:
			count2 =0
		cmd = M*math.sin(((2*math.pi)/period)*count2)

	pub.publish(Float64(cmd))
	# pub1.publish(Float64(msg[1]))
	# pub2.publish(Float64(msg[2]))
	# pub3.publish(Float64(msg[3]))
	# pub4.publish(Float64(msg[0]))
	# angles = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)

	# if (abs(angles.position[4] - msg[4]) < 0.01 and abs(angles.position[1] - msg[1]) < 0.01 and \
	#     abs(angles.position[2] - msg[2]) < 0.01 and abs(angles.position[3] - msg[3]) < 0.01 and abs(angles.position[0] - msg[0]) < 0.01):
	# 	break
	rate.sleep()
