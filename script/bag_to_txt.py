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

import argparse
import sys
import os
import rospy
import rosbag

if __name__ == '__main__':
    print 1
    # parse command line
    parser = argparse.ArgumentParser(description='''bag file to txt file''')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputtxt', help='out text file')
    args = parser.parse_args()

    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputtxt

    inbag = rosbag.Bag(args.inputbag,'r')
    outtxt = open(args.outputtxt,'w')
    outtxt.write('# text file for '+ args.inputbag + '\n# format: time stamp x y z qx qy qz qw\n')

    for topic, msg, t in inbag.read_messages():
        if topic == "/vicon_xb_node/mocap/pose":
            outtxt.write(str.format("{0:.9f} ", t.to_sec()))
            outtxt.write(str.format("{0:.9f} ", msg.pose.position.x))
            outtxt.write(str.format("{0:.9f} ", msg.pose.position.y))
            outtxt.write(str.format("{0:.9f} ", msg.pose.position.z))
            outtxt.write(str.format("{0:.9f} ", msg.pose.orientation.x))
            outtxt.write(str.format("{0:.9f} ", msg.pose.orientation.y))
            outtxt.write(str.format("{0:.9f} ", msg.pose.orientation.z))
            outtxt.write(str.format("{0:.9f}\n", msg.pose.orientation.w))