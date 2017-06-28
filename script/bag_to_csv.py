#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This script converts data from /uwb_endorange_info, /imu/data and /vicon_xb/viconPostTopic topics into a unified csv.
# All data are synchronized with UWB data. Deal with it B-)
# Input: rosbag file name.
# Output: csv file name.

import argparse
import sys
import os
import rospy
import rosbag

from uwb_driver.msg import UwbRange
from sensor_msgs.msg import Imu
from vicon_xb.msg import viconPoseMsg

# Some topics needs utf8 to decode
reload(sys)  
sys.setdefaultencoding('utf8')


if __name__ == '__main__':
    print 1
    # parse command line
    parser = argparse.ArgumentParser(description='''bag file to csv file''')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputtxt', help='out text file')
    args = parser.parse_args()

    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputtxt

    inbag = rosbag.Bag(args.inputbag,'r')
    outtxt = open(args.outputtxt,'w')
    outtxt.write('t, viconX, viconY, viconZ, viconRo, viconPi, viconYa, uwbD, uwbAnt, uwbRspdX, uwbRspdY, uwbRspdZ, imuQx, imuQy, imuQz, imuQw\n')

    # vcMsg, imuMsg, uwbMsg
    vcPublished = False
    imuPublished = False
    uwbPublished = False

    for topic, msg, t in inbag.read_messages():
        if topic == "/vicon_xb/viconPoseTopic":
            vcMsg = msg
            vcPublished = True
        elif topic == "/uwb_endorange_info":
            uwbMsg = msg
            uwbPublished = True
            if vcPublished and imuPublished and uwbPublished:
                outtxt.write(str.format("{0:.9f},", t.to_sec()))
                outtxt.write(str.format("{0:.9f},", vcMsg.pose.position.x))
                outtxt.write(str.format("{0:.9f},", vcMsg.pose.position.y))
                outtxt.write(str.format("{0:.9f},", vcMsg.pose.position.z))
                outtxt.write(str.format("{0:.9f},", vcMsg.eulers.x))
                outtxt.write(str.format("{0:.9f},", vcMsg.eulers.y))
                outtxt.write(str.format("{0:.9f},", vcMsg.eulers.z))
                outtxt.write(str.format("{0:.9f},", uwbMsg.distance))
                outtxt.write(str.format("{0:.9f},", uwbMsg.antenna))
                outtxt.write(str.format("{0:.9f},", uwbMsg.responder_location.x))
                outtxt.write(str.format("{0:.9f},", uwbMsg.responder_location.y))
                outtxt.write(str.format("{0:.9f},", uwbMsg.responder_location.z))
                outtxt.write(str.format("{0:.9f},", imuMsg.orientation.x))
                outtxt.write(str.format("{0:.9f},", imuMsg.orientation.y))
                outtxt.write(str.format("{0:.9f},", imuMsg.orientation.z))
                outtxt.write(str.format("{0:.9f}\n", imuMsg.orientation.w))
            else:
                uwbPublished = True
        elif topic == "/imu/data":
            imuMsg = msg
            imuPublished = True

