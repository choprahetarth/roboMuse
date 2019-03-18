#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
class slipControler():
	def __init__(self):
		rospy.init_node("slipController")
		nodename = rospy.get_name()
		rospy.loginfo("node %s started." % nodename)
		# subscribers
		rospy.Subscriber('imu_in', Imu, self.cb_imu)
		rospy.Subscriber('lwheel', Int32, self.cb_lwheel)
		rospy.Subscriber('rwheel', Int32, self.cb_rwheel)
		rospy.Subscriber('l_motorcmd', Float32, self.cb_l_motorcommand)
		rospy.Subscriber('r_motorcmd', Float32, self.cb_r_motorcommand)
		# publishers
		self.pub_lwheel4carto = rospy.Publisher('lwheel4carto', Int32, queue_size = 10)
		self.pub_rwheel4carto = rospy.Publisher('rwheel4carto', Int32, queue_size = 10)
		self.pub_l_motorcommand4roboclaw = rospy.Publisher('l_motorcommand4roboclaw', Float32, queue_size = 10)
		self.pub_r_motorcommand4roboclaw = rospy.Publisher('r_motorcommand4roboclaw', Float32, queue_size = 10)
		# variables
		self.variable = 1024
		self.rate = rospy.get_param("~rate", 10)
		self.lwheel4carto = 0
		self.rwheel4carto = 0
		self.l_motorcommand4roboclaw = 0.0
		self.r_motorcommand4roboclaw = 0.0
		self.original_leftEncVal = 0
		self.original_rightEncVal = 0
		# imu msg vars
		self.timeHolder = 0.0
		self.del_IMU_roll = 0.0
		self.total_IMU_roll = 0.0
		self.lateralAcc = 0.0
		self.yaw_offset = 0.0

		self.l_motorcommand = 0.0
		self.r_motorcommand = 0.0
		# filter and odom vars
		self.originalTheta = 0.0
		self.total_IMU_yaw = 0.0
		self.filteredTheta = 0.0
	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.spinOnce()
			r.sleep()
	def spinOnce(self):
		self.pub_lwheel4carto.publish(self.variable)
		self.pub_rwheel4carto.publish(self.variable)
		self.pub_l_motorcommand4roboclaw.publish(self.variable)
		self.pub_r_motorcommand4roboclaw.publish(self.variable)
	# callback functions
	def cb_lwheel(self, msg):
		self.original_leftEncVal = msg.data
		sC.calcOriginalTheta()
	def cb_rwheel(self, msg):
		self.original_rightEncVal = msg.data
		sC.calcOriginalTheta()
	def cb_imu(self, msg):
		self.timeHolder = time.time();
		self.total_IMU_yaw = math.degrees(msg.orientation.x)-self.yaw_offset
		self.del_IMU_roll = msg.orientation.z - self.total_IMU_roll
		self.total_IMU_roll = msg.orientation.z
		self.lateralAcc = msg.linear_acceleration.y
		sC.lateralSlip()
	def cb_l_motorcommand(self, msg):
		self.l_motorcommand = msg.data
	def cb_r_motorcommand(self, msg):
		self.r_motorcommand = msg.data
	# ops 4 slip detection
	def calcOriginalTheta(self):
		self.originalTheta += math.degrees(math.atan((self.original_rightEncVal-self.original_leftEncVal)/2680.24))
		print ''
		print 'original theta calculated'
		sC.weightedFilter()
	def weightedFilter(self):
		alpha4 = 1
		print 'in WF'
		print 'original Theta ========================: ',self.originalTheta
		print 'IMU yaw ===============================: ',self.total_IMU_yaw
		abs_del = abs(self.originalTheta-self.total_IMU_yaw)
		alphaQ = abs_del/(abs_del+5)
		if abs_del > 0.1 and abs_del < 15:
			filterValue = alphaQ
		elif abs_del >= 15:
			filterValue = alpha4
			print 'slip is being corrected...'
		else:
			filterValue = 0
		self.filteredTheta = (1-filterValue)*self.originalTheta + filterValue*self.total_IMU_yaw
		print 'filtered Theta ===========================: ',self.filteredTheta
		print 'outta weightedF'
		sC.peakDetection()
	def lateralSlip(self):
		if abs(self.lateralAcc) >= 0.8:
			print 'sudden lateral slip detected'
	def peakDetection(self):
		del_Time = time.time() - self.timeHolder
		print 'del time ============%r' % del_Time
		self.timeHolder = time.time()
		der_del_roll = self.del_IMU_roll / del_Time
		print 'der_del_roll=========== %f',der_del_roll
		if abs(der_del_roll) > 0.1:
			print 'sudden roll Detected..'
		print 'outta peakDet'
if __name__ == '__main__':
	try:
		sC = slipControler()
		sC.spin()
	except rospy.ROSInterruptException:
		pass
