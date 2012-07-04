#!/usr/bin/env python
import roslib; roslib.load_manifest('thymio_navigation_driver')
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyToTwist(object):
	def __init__(self):
		rospy.Subscriber('/joy', Joy, self.joy_cb)
		self.cmdvel_pub = rospy.Publisher('/cmd_vel', Twist)
	
	def joy_cb(self, joy):
		tw = Twist()
		tw.linear.x = 0.15*joy.axes[1]
		tw.angular.z = 2.*joy.axes[0]
		self.cmdvel_pub.publish(tw)

## Create a ROS node and instantiate the class.
def main():
	'''Create a ROS node and instantiate the class.'''
	try:
		rospy.init_node('joy_to_twist')
		jtt = JoyToTwist()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__== '__main__':
	main()
	

