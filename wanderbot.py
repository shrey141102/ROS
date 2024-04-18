#!/usr/bin/env python3
import rospy
from math import pi
from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
	global g_range_ahead
	g_range_ahead = msg.ranges[len(msg.ranges)//2]
	print(g_range_ahead)

g_range_ahead = 1 # anything to start

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	print("range", g_range_ahead)
	if driving_forward:
		print("driving forward. ahead: ", g_range_ahead)
		if (g_range_ahead < 1 or rospy.Time.now() > state_change_time):
			driving_forward = False
			state_change_time = rospy.Time.now() + rospy.Duration(2)
	else: # we're not driving_forward
		print("not driving")
		if rospy.Time.now() > state_change_time:
			driving_forward = True # we're done spinning, time to go forward
			state_change_time = rospy.Time.now() + rospy.Duration(2)
	twist = Twist()
	if driving_forward:
		twist.linear.x = 0.2
	else:
		#twist.linear.x = 1
		duration=10
		twist.angular.z =pi*2/4/duration

	cmd_vel_pub.publish(twist)
	rate.sleep()