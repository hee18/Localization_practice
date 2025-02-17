#!/usr/bin/env python

import rospy
from ublox_msgs.msg import NavPVT
import random
import math

class NAVPVT:

    def __init__(self):
        rospy.init_node("loc_hackers")
        
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.callback)
        self.hacked_msg = NavPVT()
        self.hdg_hacked = False
        self.pos_hacked = False

        self.pub_loc = rospy.Publisher("/localization", NavPVT, queue_size=1)

        self.last_hdg_hack_time = rospy.Time.now()
        self.last_pos_hack_time = rospy.Time.now()

        self.hdg_hack_interval = rospy.Duration(10)  # 헤딩 오차 추가 타이머 (10s)
        self.pos_hack_interval = rospy.Duration(15)  # 위치 오차 추가 타이머 (15s)

    def callback(self, msg):
        self.hacked_msg.iTOW = msg.iTOW
        self.hacked_msg.year = msg.year
        self.hacked_msg.month = msg.month
        self.hacked_msg.day = msg.day
        self.hacked_msg.hour = msg.hour
        self.hacked_msg.min = msg.min
        self.hacked_msg.sec = msg.sec
        self.hacked_msg.valid = msg.valid
        self.hacked_msg.tAcc = msg.tAcc
        self.hacked_msg.nano = msg.nano
        self.hacked_msg.fixType = msg.fixType
        self.hacked_msg.flags = msg.flags
        self.hacked_msg.flags2 = msg.flags2
        self.hacked_msg.numSV = msg.numSV
        self.hacked_msg.height = msg.height
        self.hacked_msg.hMSL = msg.hMSL
        self.hacked_msg.hAcc = msg.hAcc
        self.hacked_msg.vAcc = msg.vAcc
        self.hacked_msg.velN = msg.velN
        self.hacked_msg.velE = msg.velE
        self.hacked_msg.velD = msg.velD
        self.hacked_msg.gSpeed = msg.gSpeed
        self.hacked_msg.sAcc = msg.sAcc
        self.hacked_msg.headAcc = msg.headAcc
        self.hacked_msg.pDOP = msg.pDOP
        self.hacked_msg.reserved1 = msg.reserved1
        self.hacked_msg.headVeh = msg.headVeh
        self.hacked_msg.magDec = msg.magDec
        self.hacked_msg.magAcc = msg.magAcc

        self.check_hack_timers()

        if self.pos_hacked:
            theta = random.uniform(0, 2 * math.pi)
            r = 100  # 오차 100m

            lat_offset = (r * math.sin(theta)) / 111320  # 1도 ~= 111320m
            lon_offset = (r * math.cos(theta)) / (111320 * math.cos(math.radians(msg.lat * 1e-7)))

            self.hacked_msg.lat = msg.lat + int(lat_offset * 1e7)
            self.hacked_msg.lon = msg.lon + int(lon_offset * 1e7)
            self.pos_hacked = False
        else:
            self.hacked_msg.lon = msg.lon
            self.hacked_msg.lat = msg.lat

        if self.hdg_hacked:
            heading_offset = random.choice([-30, 30]) * 1e5  # 오차 +-30 deg
            self.hacked_msg.heading = int(msg.heading + heading_offset )
            self.hdg_hacked = False
        else:
            self.hacked_msg.heading = msg.heading
    

    def check_hack_timers(self):
        """ 15초마다 위치 오차, 10초마다 헤딩 오차 자동으로 적용 """
        current_time = rospy.Time.now()

        if current_time - self.last_hdg_hack_time >= self.hdg_hack_interval:
            self.hdg_hacked = not self.hdg_hacked
            self.last_hdg_hack_time = current_time
            rospy.loginfo(f"[LOC_HACK] Heading hack toggled: {self.hdg_hacked}")

        if current_time - self.last_pos_hack_time >= self.pos_hack_interval:
            self.pos_hacked = not self.pos_hacked
            self.last_pos_hack_time = current_time
            rospy.loginfo(f"[LOC_HACK] Position hack toggled: {self.pos_hacked}")


    def main(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            self.pub_loc.publish(self.hacked_msg)
            
            rate.sleep()

if __name__ == "__main__":
    navpvt = NAVPVT()
    navpvt.main()
