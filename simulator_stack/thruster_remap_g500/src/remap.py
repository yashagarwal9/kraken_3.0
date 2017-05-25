#!/usr/bin/python
import roslib;roslib.load_manifest('thruster_remap_g500')

import rospy
import kraken_msgs
import geometry_msgs.msg as gs
from kraken_msgs.msg._forceData6Thruster import forceData6Thruster
from kraken_msgs.msg._forceData4Thruster import forceData4Thruster

from kraken_msgs.msg._thrusterData4Thruster import thrusterData4Thruster
from kraken_msgs.msg._thrusterData6Thruster import thrusterData6Thruster


from std_msgs.msg import Float64MultiArray
from resources import topicHeader
from std_srvs.srv import Empty

import numpy as np
from kraken_msgs.msg import imuData, dvlData
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_matrix
from tf.transformations import quaternion_multiply
import tf
from sensor_msgs.msg import Imu
from underwater_sensor_msgs.msg import DVL

thrusters_topic="/g500/thrusters_input"
pose_topic ='/g500/pose'
pub = rospy.Publisher(thrusters_topic, Float64MultiArray, queue_size=10)
imu_pub= rospy.Publisher(topicHeader.SENSOR_IMU, imuData, queue_size=10)
dvl_pub= rospy.Publisher(topicHeader.SENSOR_DVL, dvlData, queue_size=10)
pose_pub= rospy.Publisher(topicHeader.SIMULATOR_POSE, gs.Pose, queue_size=10)
twist_pub= rospy.Publisher(topicHeader.SIMULATOR_TWIST, gs.Twist, queue_size=10)

rospy.wait_for_service('/dynamics/reset')
reset=rospy.ServiceProxy('/dynamics/reset', Empty)

def remapandpublish(data):
	'''
	Mapping of simulator thrusters and their locations:
	Simulator thruster[0] - Right
	Simulator thruster[1] - Left
	Simulator thruster[2] - forward
	Simulator thruster[3] - Back
	Simulator thruster[4] - Sway

	forward Thrust - positive, Backward - Negative
	Upward Thrust  - positive, Downward - Negative
	'''
	new_thrusters=[0]*5

	msg = Float64MultiArray()
	new_thrusters[0]= data.data[1]
 	new_thrusters[1]= data.data[0]
	new_thrusters[2]= data.data[5]
	new_thrusters[3]= data.data[4]
	new_thrusters[4]= data.data[2]+data.data[3]
	msg.data=new_thrusters
	pub.publish(msg)

def remapandpublish4(data):
	new_thrusters=[0]*5

	msg = Float64MultiArray()
	new_thrusters[0]=data.data[1]
	new_thrusters[1]=data.data[0]
	new_thrusters[4]=-data.data[2]-data.data[3]

	msg.data=new_thrusters
	pub.publish(msg)

def remapImuAndPublish(dataIn):
	'''
	Converts the message from sensors_msgs/Imu to kraken_msgs/imuData
	'''

	newImuData = imuData()
	newImuData.data = [0] * 13

	## Roll, pitch and yaw
	orientation = dataIn.orientation
	rpy = np.array(euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])) * 180 / np.pi

	newImuData.data[0] = rpy[0]
	newImuData.data[1] = rpy[1]
	newImuData.data[2] = rpy[2]

	## ax, ay, az
	newImuData.data[3] = dataIn.linear_acceleration.x
	newImuData.data[4] = dataIn.linear_acceleration.y
	newImuData.data[5] = dataIn.linear_acceleration.z

	imu_pub.publish(newImuData)

def remapDvlAndPublish(dataIn):
	'''
	Convert the message underwater_sensor_msgs/DVL to kraken_msgs/DvlData
	'''

	newDvlData = dvlData()
	newDvlData.data = [0] * 10

	newDvlData.data[7] = dataIn.depth

	## vx, vy and vz

	newDvlData.data[3] = dataIn.bi_x_axis
	newDvlData.data[4] = dataIn.bi_y_axis
	newDvlData.data[5] = dataIn.bi_z_axis

	dvl_pub.publish(newDvlData)

def publish_pose(data):

	roll = -np.pi
	pitch = 0
	yaw = -np.pi/2

	R = euler_matrix(roll, pitch, yaw, 'sxyz').transpose()
	r = R.dot(np.array([data.position.x, data.position.y, data.position.z, 0]))

	q1 = quaternion_from_euler(roll, pitch , yaw, 'sxyz')
	q2 = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
	q3 = quaternion_from_euler(0, np.pi, 0, 'sxyz')

	q = quaternion_multiply(q1, q2)
	q = quaternion_multiply(q, q3)
	# listener = tf.TransformListener()

	# flag = 1
	# while flag:
	# 	try:
	# 		(trans,q) = listener.lookupTransform("/odom", "/body", rospy.Time.now())
	# # 		#(trans,q3) = listener.lookupTransform("/girona500/base_link", "/body", rospy.Time(0))
	# 		flag = 0
	# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 		continue

	msg = gs.Pose()
	msg.position = gs.Point(r[0], r[1], r[2])
	msg.orientation = gs.Quaternion(q[0], q[1], q[2], q[3])

	pose_pub.publish(msg)

def publish_twist(data):
	roll = 0
	pitch = np.pi
	yaw = 0
	R = euler_matrix(roll, pitch, yaw, 'sxyz')

	v = R.dot(np.array([data.linear.x, data.linear.y, data.linear.z, 0]))
	omega = R.dot(np.array([data.angular.x, data.angular.y, data.angular.z, 0]))

	msg = gs.Twist()
	msg.linear = gs.Vector3(v[0], v[1], v[2])
	msg.angular = gs.Vector3(omega[0], omega[1], omega[2])

	twist_pub.publish(msg)


rospy.init_node('remapper', anonymous=False)

rospy.Subscriber('/g500/imu', Imu, remapImuAndPublish)
rospy.Subscriber('/g500/dvl', DVL, remapDvlAndPublish)

# rospy.Subscriber(topicHeader.CONTROL_PID_THRUSTER4, thrusterData4Thruster, remapandpublish4)
rospy.Subscriber(topicHeader.CONTROL_PID_THRUSTER6, thrusterData6Thruster, remapandpublish)
rospy.Subscriber(pose_topic,gs.Pose,publish_pose)
rospy.Subscriber("/g500/twist", gs.Twist, publish_twist)

rospy.spin()
