#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool, Float32


class Safety(object):

	def __init__(self):
	
		self.speed = 0.5
#		self.odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
		self.scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
		self.drive = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 10)
#		self.brakePub = rospy.Publisher("brake", AckermannDriveStamped, queue_size = 10)
#		self.brakeBoolPub = rospy.Publisher("brake_bool", Bool, queue_size = 10)
		self.TTCPub = rospy.Publisher("TTC", Float32, queue_size = 10)
		self.TTCminPub = rospy.Publisher("minTTC", Float32, queue_size = 10) 
		self.ackMsg = AckermannDriveStamped()
		self.ackMsg.speed = 0.5
		self.drive.publish(self.ackMsg) 
		
#	def odom_callback(self, odom_msg):

#		self.speed = odom_msg.twist.twist.linear.x
		
	def scan_callback(self, scan_msg):
	
		minAngle = scan_msg.angle_min
		incAngle = scan_msg.angle_increment
		ranges = scan_msg.ranges
		speed = self.speed
		minTTC = .4
		smallestTTC = 1
		
		smallRange = min(ranges)
		index = ranges.index(smallRange)
		angle = minAngle + incAngle * index
		rDot = np.cos(angle) * speed
		if rDot > 0 and smallRange/rDot < smallestTTC: 
			smallestTTC = smallRange/rDot
			
		self.TTCPub.publish(smallestTTC)
		self.TTCminPub.publish(minTTC) 
			
		if smallestTTC < minTTC: 
#			brake_bool = True
#			self.brakeBoolPub.publish(brake_bool)
			self.ackMsg.speed = 0
			self.drive.publish(self.ackMsg)
		else: 
#			brake_bool = False
#			self.brakeBoolPub.publish(brake_bool)

		
				
def main():
	rospy.init_node('safety_node')
	sn = Safety()
	rospy.spin()
if __name__ == '__main__':
	main()
