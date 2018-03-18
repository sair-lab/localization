#!/usr/bin/env python
# license removed for brevity

import rospy
from uwb_driver.msg import UwbRange
from vicon_xb.msg import viconPoseMsg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber
from bitcraze_lps_estimator.msg import RangeArray

import tf

ps = PoseStamped()
ps.pose.orientation.w = 1
ps.pose.orientation.x = 0
ps.pose.orientation.y = 0
ps.pose.orientation.z = 0
ps.pose.position.x = 0
ps.pose.position.y = 0
ps.pose.position.z = 0



def callback1(msg):

   global i  
  # for i in range(6):
  
   uwb_msg = UwbRange()
   uwb_msg.header.stamp=rospy.Time.now()
   uwb_msg.header.frame_id="range"
   uwb_msg.requester_id= 8
   uwb_msg.responder_id= i%6
   uwb_msg.distance=msg.ranges[i%6] 
   uwb_msg.distance_err=0.1273
   uwb_msg.antenna=0
   uwb_msg.responder_location.x=0
   uwb_msg.responder_location.y=0
   uwb_msg.responder_location.z=0
   i=i+1

   uwb_pub.publish(uwb_msg);

    # rospy.sleep(0.01)
   		
def callback2(msg):

   ps.pose.position.x = msg.pose.position.x
   ps.pose.position.y = msg.pose.position.y
   ps.pose.position.z = msg.pose.position.z
   ps.header.frame_id = "/world"
   ps.header.stamp = rospy.Time.now()

   pose_pub.publish(ps)

   br = tf.TransformBroadcaster()
   br.sendTransform(
            (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z),
            (ps.pose.orientation.x, ps.pose.orientation.y,
                ps.pose.orientation.z, ps.pose.orientation.w),
            rospy.Time.now(),
            rospy.get_namespace() + "base_link",
            "world")





if __name__ == '__main__':
    
   rospy.init_node('lps_range_node', anonymous=True)
  
   i=0

   rospy.Subscriber("/crazyflie1/ranging", RangeArray, callback1)


   rospy.Subscriber("/localization_node/realtime/pose1", PoseStamped, callback2)



   uwb_pub = rospy.Publisher('/lpsrange', UwbRange, queue_size=10)

   pose_pub = rospy.Publisher(rospy.get_namespace() + "pose",
                               PoseStamped, queue_size=10)

   # rospy.wait_for_service('update_params')
   # update_params = rospy.ServiceProxy('update_params', UpdateParams)

   # rospy.loginfo("Setting anchor position ...")

   # n_anchors = rospy.get_param("n_anchors")
   # for i in range(n_anchors):
   #      position = rospy.get_param("anchor{}_pos".format(i))
   #      rospy.loginfo("Anchor {} at {}".format(i, position))
   #      name = "anchorpos/anchor{}".format(i)
   #      rospy.set_param(name + "x", position[0])
   #      rospy.set_param(name + "y", position[1])
   #      rospy.set_param(name + "z", position[2])
   #      update_params([name + 'x', name + 'y', name + 'z'])

   # if rospy.has_param("anchorpos/enable"):
   #       rospy.set_param("anchorpos/enable", 1)
   #       update_params(["anchorpos/enable"])


   rospy.spin()