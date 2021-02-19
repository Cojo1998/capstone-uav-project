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
import cv2
import sys
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

class image_encoder:

    def __init__(self):
        self.encodeLeft = rospy.Publisher("/stereo/left/image_raw", Image, queue_size = 10)
        self.encodeRight = rospy.Publisher("/stereo/right/image_raw", Image, queue_size = 10)
        self.leftFeed = rospy.Subscriber("/left/image_encode", Image, self.callbackLeft)
        self.rightFeed = rospy.Subscriber("/right/image_encode", Image, self.callbackRight)
        self.bridge = CvBridge()

    def callbackLeft(self,data):
        try:
            image_left = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        bgrLeft = cv2.cvtColor(image_left, cv2.COLOR_BGRA2BGR)

        try:
            self.encodeLeft.publish(self.bridge.cv2_to_imgmsg(bgrLeft, "bgr8"))
        except CvBridgeError as e:
            print(e)

        #cv2.imshow("left", image_left)
        #cv2.waitKey(3)

    def callbackRight(self,data):
        try:
            image_right = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        bgrRight = cv2.cvtColor(image_right, cv2.COLOR_BGRA2BGR)

        try:
            self.encodeRight.publish(self.bridge.cv2_to_imgmsg(bgrRight, "bgr8"))
        except CvBridgeError as e:
            print(e)

        #cv2.imshow("right", image_right)
        #cv2.waitKey(3)

def main(args):
    rospy.init_node('encoded_data', anonymous=False)
    ie = image_encoder()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down")
        pass

if __name__ == '__main__':
    main(sys.argv)