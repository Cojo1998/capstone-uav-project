#!/usr/bin/env python3
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
from sensor_msgs.msg import Imu

client = airsim.MultirotorClient(ip="10.0.0.53") #connect to the AirsSim simulator #192.168.210.7
client.confirmConnection() #confirm connection

def talker():
    #pubImu = rospy.Publisher("/imuTest", Imu, queue_size = 10)
    pubLeft = rospy.Publisher("/left/image_encode", Image, queue_size = 10)
    pubRight = rospy.Publisher("/right/image_encode", Image, queue_size = 10)
    rospy.init_node('airsim_data', anonymous=False)
    bridge = CvBridge()
    #imu = Imu()
    counter = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rawImageLeft = client.simGetImage("2", airsim.ImageType.Scene)
        pngLeft = cv2.imdecode(airsim.string_to_uint8_array(rawImageLeft), cv2.IMREAD_UNCHANGED)
        cv2.imshow("Left Cam1", pngLeft)
        try:
            pubLeft.publish(bridge.cv2_to_imgmsg(pngLeft))
        except CvBridgeError as e:
            print(e)

        rawImageRight = client.simGetImage("1", airsim.ImageType.Scene)
        pngRight = cv2.imdecode(airsim.string_to_uint8_array(rawImageRight), cv2.IMREAD_UNCHANGED)
        cv2.imshow("Right Cam1", pngRight)
        try:
            pubRight.publish(bridge.cv2_to_imgmsg(pngRight, encoding="passthrough"))
        except CvBridgeError as e:
            print(e)

        #counter += 1
        #imu_data = client.getImuData(imu_name = "", vehicle_name = "")
        #imu.header.seq = counter
        #imu.header.frame_id = "imu4"
        #imu.header.stamp = rospy.Time.now()
        #imu.angular_velocity.x = imu_data.angular_velocity.x_val
        #imu.angular_velocity.y = imu_data.angular_velocity.y_val
        #imu.angular_velocity.z = imu_data.angular_velocity.z_val
        #imu.linear_acceleration.x = imu_data.linear_acceleration.x_val
        #imu.linear_acceleration.y = imu_data.linear_acceleration.y_val
        #imu.linear_acceleration.z = imu_data.linear_acceleration.z_val
        #imu.orientation.x = imu_data.orientation.x_val
        #imu.orientation.y = imu_data.orientation.y_val
        #imu.orientation.z = imu_data.orientation.z_val
        #imu.orientation.w = imu_data.orientation.w_val
        #pubImu.publish(imu)

        rate.sleep()

        if cv2.waitKey(1) & 0xff ==ord(' '):
            break


if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass

