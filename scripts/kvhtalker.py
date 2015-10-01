#!/usr/bin/env python
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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
#from geometric_msgs.msg import Vector3
#from geometric_msgs.msg import Quaternion
import tf

import serial
import time
import struct
import sys
import os
import math
import ast
import redis
import binascii

# current Millisecond function
def current_milli_time():
    """
    Returns current timestamp in millisecond
    @retval millisecond
    """
    return int(round(time.time() * 1000))


def talker():

    port = "/dev/ttyUSB1"  # Serial port
    ser = serial.Serial(port, 921600, bytesize=8, timeout=0, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None)

    pre_bytes = ''  # Global variable to save previous bytes, if
    is_valid = False  # Flag to check bytes validity
    buff_count = 0  # add counter if buffer size exceed 36 bytes, I used for my code checking
    OX = 0  # Orientation X
    OY = 0  # Orientation Y
    OZ = 0  # Orientation Z
    VX = 0  # Vehicle X position
    VY = 0  # Vehicle Y position

    VDist = 0

    pub  = rospy.Publisher('chatter', String, queue_size=10)
    pub2 = rospy.Publisher('microstrain/imu', Imu, queue_size=10)

    imu_msg = Imu()
    imu_msg.orientation_covariance = [0.0012250000000000002, 0.0, 0.0, 0.0, 0.0012250000000000002, 0.0, 0.0, 0.0, 0.0012250000000000002]
    imu_msg.angular_velocity_covariance = [6.25e-06, 0.0, 0.0, 0.0, 6.25e-06, 0.0, 0.0, 0.0, 6.25e-06]
    imu_msg.linear_acceleration_covariance = [0.00031329000000000003, 0.0, 0.0, 0.0, 0.00031329000000000003, 0.0, 0.0, 0.0, 0.00031329000000000003]

    rospy.init_node('kvh_talker', anonymous=True)

    pre_tstamp = rospy.get_time()#current_milli_time()  # previous timestamp variable for DeltaT calculation
    count_stable = 0

    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():

        bytesToRead = ser.inWaiting()  # make serial in waiting for read data
        data = ser.read(3600)  # read serial data
        #print type(data)
        data_len = len(data)  # store data length

        if data_len == 36:  # data length is 36 bytes
          #print 'Got:',data_len, binascii.hexlify(data)
          imu_data = data  # save to imu_data variable
          is_valid = True  # mark as valid bytes
          pre_bytes = ''  # make pre_bytes empty
        elif data_len < 36:  # if data is less than 36 bytes, need to truncate, next bytes
          pre_bytes = pre_bytes+data  # truncate bytes
          if len(pre_bytes) == 36:  # if truncated bytes is 36 mark as valid data
            print 'Got:',len(pre_bytes), binascii.hexlify(pre_bytes)
            imu_data = pre_bytes
            pre_bytes = ''
            is_valid = True
        elif data_len > 36:  # check how many bytes exceed 36 bytes length
          buff_count +=1

        if(is_valid):
          _structure = struct.Struct('> ' + '4s 6f 2B 1h 4s')   # Bytes structure. 4 Hex, 6 Float (3 Orienattaion and 3 Accelorometer), 2Bytes represets temperature, 1 Short Int  represents data validity,  4 Hex values of cyclic data
          #print len(imu_data)
          unpacked_data = _structure.unpack(imu_data)  # unpack bytes based on structure defined before

          #print "\n", unpacked_data[7]
          if (unpacked_data[0].encode('hex') == 'fe81ff55'):  # check bytes are valid based on structure
            #print ('X: %f \nY: %f \nZ: %f \n' %(unpacked_data[1],unpacked_data[2],unpacked_data[3]))
            #print ('x: %f \ny: %f \nz: %f \n' %(unpacked_data[4],unpacked_data[5],unpacked_data[6]))

            #store orientation rate
            dox = math.radians(unpacked_data[1])
            doy = math.radians(unpacked_data[2])
            doz = math.radians(unpacked_data[3])

            #hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hello_str)
            #pub.publish(hello_str)

            _ts = rospy.get_time()
            deltaT = _ts - pre_tstamp
            pre_tstamp = _ts

            #ignore first readings to allow comm stabilize, before integrating orientation
            if count_stable > 50:
              OX  = OX + dox*deltaT
              OY  = OY + doy*deltaT
              OZ  = OZ + doz*deltaT

            imu_msg.header.frame_id = 'microstrain'
            imu_msg.header.stamp = rospy.Time.now()


            #convert to quaternion format
            quaternion = tf.transformations.quaternion_from_euler(OX, OY, OZ)

            imu_msg.orientation.x =  quaternion[0]#1
            imu_msg.orientation.y =  quaternion[1]#2
            imu_msg.orientation.z =  quaternion[2]#3
            imu_msg.orientation.w =  quaternion[3]#4

            imu_msg.angular_velocity.x = dox#5
            imu_msg.angular_velocity.y = doy#6
            imu_msg.angular_velocity.z = doz#7

            imu_msg.linear_acceleration.x = unpacked_data[4]#8
            imu_msg.linear_acceleration.y = unpacked_data[5]#9
            imu_msg.linear_acceleration.z = unpacked_data[6]#10

            #print imu_msg.orientation_covariance
            pub2.publish(imu_msg)

            if count_stable < 51:
              count_stable = count_stable + 1
          else:
            is_valid = False
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
