#!/usr/bin/env python3
#!/usr/bin/env python2
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$


import rospy
import airsim
import cv2
import numpy as np
import time
import sys

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image

client = airsim.MultirotorClient(ip="192.168.210.7") #connect to the AirsSim simulator
client.confirmConnection() #confirm connection

def talker():
    pub = rospy.Publisher("/cam0/image_raw", Image, queue_size = 10)
    rospy.init_node('leftStereo', anonymous=False)
    bridge = CvBridge()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rawImage = client.simGetImage("2", airsim.ImageType.Scene)
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
        cv2.imshow("Left Cam1", png)
        pub.publish(bridge.cv2_to_imgmsg(png))
        imu_data = client.getImuData(imu_name = "", vehicle_name = "")
        print(imu_data)

        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

        if cv2.waitKey(1) & 0xff ==ord(' '):
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass