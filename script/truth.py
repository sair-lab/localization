#!/usr/bin/env python
# license removed for brevity

import rospy
from uwb_driver.msg import UwbRange
from vicon_xb.msg import viconPoseMsg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber


filename1 = '/home/xufang/experiment_data/py_truth_data.txt'
filename2 = '/home/xufang/experiment_data/py_estimation_data.txt'
filename3 = '/home/xufang/experiment_data/st_data.txt'


def callback1(msg):

   t = msg.header.stamp
   t1 = msg.pose.position
   t2 = msg.pose.orientation
   
   f = open(filename1,'a')
   f.write('%.9f %f %f %f %f %f %f %f\n' % ((t.secs+0.000000001*t.nsecs),t1.x,t1.y,t1.z,t2.x,t2.y,t2.z,t2.w,))
   f.close()


def callback2(msg):

   t = msg.header.stamp
   t1 = msg.pose.position
   t2 = msg.pose.orientation
   
   f = open(filename2,'a')
   f.write('%.9f %f %f %f %f %f %f %f\n' % ((t.secs+0.000000001*t.nsecs),t1.x,t1.y,t1.z,t2.x,t2.y,t2.z,t2.w,))
   f.close()


def callback3(msg):

   t = msg.header.stamp
   t1= msg.responder_id
   t2= msg.distance

   f = open(filename3,'a')  
   f.write('%.9f %d %f \n' % ((t.secs+0.000000001*t.nsecs),t1,t2,))
   f.close()




if __name__ == '__main__':
    
   rospy.init_node('truth_node', anonymous=True)
   f1 = open(filename1,'a')
   f1.seek(0)
   f1.truncate()
   f1.close()

   f2 = open(filename2,'a')
   f2.seek(0)
   f2.truncate()
   f2.close()


   f3 = open(filename3,'a')
   f3.seek(0)
   f3.truncate()
   f3.close()


   # rospy.Subscriber("/vicon_xb/viconPoseTopic", viconPoseMsg, callback1)
   rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback1)
   rospy.Subscriber("/localization_node/realtime/pose1", PoseStamped, callback2) 
   rospy.Subscriber("/uwb_endorange_info", UwbRange, callback3)


	
   rospy.spin()


