#!/usr/bin/env python
#kate: replace-tabs off; tab-width 4; indent-width 4; indent-mode normal
import roslib; roslib.load_manifest('thymio_navigation_driver')
import rospy
from asebaros.msg import AsebaAnonymousEvent
from asebaros.msg import AsebaEvent
from asebaros.srv import LoadScripts
from asebaros.srv import GetNodeList
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from math import sin,cos,atan2
import time

BASE_WIDTH = 95     # millimeters
MAX_SPEED = 500     # units
SPEED_COEF = 2.93   # 1mm/sec corresponds to X units of real thymio speed

class ThymioDriver():
	# ======== class initialization ======== 
	def __init__(self):
		rospy.init_node('thymio')
		# initialize parameters
		self.x = 0
		self.y = 0
		self.th = 0
		self.then = rospy.Time.now()
		self.odom = Odometry(header=rospy.Header(frame_id='odom'),child_frame_id='base_link')
		
		# load script on the Thymio
		rospy.wait_for_service('/aseba/load_script')
		load_script = rospy.ServiceProxy('/aseba/load_script',LoadScripts)
		script_filename = roslib.packages.get_pkg_dir('thymio_navigation_driver') + '/aseba/thymio_ros.aesl'
		load_script(script_filename)
		
		# subscribe to topics
		rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel)
		rospy.Subscriber('/aseba/events/odometry', AsebaEvent, self.on_aseba_odometry_event)
		self.aseba_pub = rospy.Publisher('/aseba/events/set_speed', AsebaEvent)
		self.odom_pub = rospy.Publisher('odom',Odometry)
		self.odom_broadcaster = TransformBroadcaster()
	
	# ======== we send the speed to the aseba running on the robot  ======== 
	def set_speed(self,values):
		self.aseba_pub.publish(AsebaEvent(rospy.get_rostime(),0,values))
	
	# ======== stop the robot safely ======== 
	def shutdown(self):
		self.set_speed([0,0])
	
	# ======== processing odometry events received from the robot ======== 
	def on_aseba_odometry_event(self,data): 
		now = data.stamp
		dt = (now-self.then).to_sec()
		self.then = now
		dsl = (data.data[0]*dt)/SPEED_COEF # left wheel delta in mm
		dsr = (data.data[1]*dt)/SPEED_COEF # right wheel delta in mm
		ds = ((dsl+dsr)/2.0)/1000.0      # robot traveled distance in meters
		dth = atan2(dsr-dsl,BASE_WIDTH)  # turn angle

		self.x += ds*cos(self.th+dth/2.0)
		self.y += ds*sin(self.th+dth/2.0)
		self.th+= dth

		# prepare tf from base_link to odom 
		quaternion = Quaternion()
		quaternion.z = sin(self.th/2.0)
		quaternion.w = cos(self.th/2.0)

		# prepare odometry
		self.odom.header.stamp = rospy.Time.now() # OR TO TAKE ONE FROM THE EVENT?
		self.odom.pose.pose.position.x = self.x
		self.odom.pose.pose.position.y = self.y
		self.odom.pose.pose.position.z = 0
		self.odom.pose.pose.orientation = quaternion
		self.odom.twist.twist.linear.x = ds/dt
		self.odom.twist.twist.angular.z = dth/dt

		# publish odometry
		self.odom_broadcaster.sendTransform((self.x,self.y,0),(quaternion.x,quaternion.y,quaternion.z,quaternion.w),self.then,"base_link","odom")
		self.odom_pub.publish(self.odom)
	
	# ======== processing events received from the robot  ======== 
	def on_cmd_vel(self,data):
		x = data.linear.x * 1000.0 # from meters to millimeters 
		x = x * SPEED_COEF # to thymio units
		th = data.angular.z * (BASE_WIDTH/2) # in mm
		th = th * SPEED_COEF # in thymio units
		k = max(abs(x-th),abs(x+th))
		# sending commands higher than max speed will fail
		if k > MAX_SPEED:
			x = x*MAX_SPEED/k; th = th*MAX_SPEED/k
		self.set_speed([int(x-th),int(x+th)])

	# ======== ======== ======== ======== ======== ======== ========      
	def control_loop(self):	
		rospy.on_shutdown(self.shutdown) # on shutdown hook
		while not rospy.is_shutdown():
			rospy.spin()

def main():
	try:
		robot = ThymioDriver()
		robot.control_loop()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
	