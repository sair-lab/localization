#!/usr/bin/env python
# license removed for brevity

import rospy
from uwb_driver.msg import Uwbrange
from vicon_xb.msg import viconPoseMsg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber





def callback1(msg):

     
   for i in range(6):

   		Uwbrange uwb_msg
  		   uwb_msg.Header.stamp = rospy.Time.now()
   		uwb_msg.Header.frame_id = "range";
   		uwb_msg.requester_id= 8;
   		uwb_msg.responder_id= i;
   		uwb_msg.distance=mas.ranges[i]; 
   		uwb_msg.distance_err=0.1273;
   		uwb_msg.antenna=0;
   		uwb_msg.responder_location.x=0;
   		uwb_msg.responder_location.y=0;
   		uwb_msg.responder_location.z=0;
   		uwb_pub.publish(uwb_msg);
   		
         ,,



if __name__ == '__main__':
    
   rospy.init_node('lps_range_node', anonymous=True)
   
   rospy.Subscriber("/crazyflie1/ranging", Float32[], callback1)

   uwb_pub = rospy.Publisher('/lpsrange', Uwbrange, queue_size=10)


   rospy.spin()