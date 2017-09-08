#!/usr/bin/env python
# license removed for brevity

import rospy
from uwb_driver.msg import UwbRange
from vicon_xb.msg import viconPoseMsg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber


	
def velocity_estimation(current_pose,truth):

	global last_pose 
	global last_velocity_estimation

	velocity_estimation.x = 0.9*last_velocity_estimation.x + 0.1*(current_pose.pose.position.x - last_pose.pose.position.x)/(current_pose.header.stamp.to_sec() - last_pose.header.stamp.to_sec())
	velocity_estimation.y = 0.9*last_velocity_estimation.y + 0.1*(current_pose.pose.position.y - last_pose.pose.position.y)/(current_pose.header.stamp.to_sec() - last_pose.header.stamp.to_sec())
	velocity_estimation.z = 0.9*last_velocity_estimation.z + 0.1*(current_pose.pose.position.z - last_pose.pose.position.z)/(current_pose.header.stamp.to_sec() - last_pose.header.stamp.to_sec())
	
	last_velocity_estimation.x = velocity_estimation.x
	last_velocity_estimation.y = velocity_estimation.y
	last_velocity_estimation.z = velocity_estimation.z

	last_pose = current_pose

	estimation_error.x = last_velocity_estimation.x - truth.vel.x
	estimation_error.y = last_velocity_estimation.y - truth.vel.y
	estimation_error.z = last_velocity_estimation.z - truth.vel.z

	pub.publish(estimation_error)

if __name__ == '__main__':

	rospy.init_node('velocity_node', anonymous=True)
	rate = rospy.Rate(10)
	pub = rospy.Publisher('velocity_error', Vector3, queue_size=10)

	last_pose = PoseStamped()
	last_velocity_estimation = Vector3()
	estimation_error = Vector3()

	tss = ApproximateTimeSynchronizer([Subscriber("/localization_node/realtime/pose", PoseStamped), 
									   Subscriber("/vicon_xb/viconPoseTopic", viconPoseMsg)],10,0.1)
	tss.registerCallback(velocity_estimation)

	rospy.spin()
