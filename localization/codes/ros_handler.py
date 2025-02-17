import rospy

from localization_practice.msg import *
from ublox_msgs.msg import NavPVT
from geometry_msgs.msg import Pose
from pyproj import Proj, Transformer

class ROSHandler():
    def __init__(self):
        rospy.init_node('localization', anonymous=False)
        self.set_values()
        self.set_protocol()
        self.set_params()

    def set_values(self):
        self.base_lla = None
        self.transformer = None
        self.llh = [None, None]
        
        self.ori_pos = [None, None]
        self.ori_hdg = [None, None]

        self.noisy_pos = [None, None]
        self.noisy_hdg = None

        self.noisy_pos_last = [None, None]
        self.noisy_hdg_last = None

        self.can_velocity = None
        self.can_steer = None
        self.corr_can_velocity = None

        self.can_velocity_last = None
        self.can_steer_last = None
        self.corr_can_velocity_last = None

        self.loc_on = False
        self.hAcc = None
        self.headAcc = None


    def set_protocol(self):
        rospy.Subscriber('/ublox/navpvt', NavPVT, self.ori_cb)
        rospy.Subscriber('/localization', NavPVT, self.loc_cb)
        rospy.Subscriber('/CANOutput', CANOutput, self.canoutput_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        
        self.corr_pose_pub = rospy.Publisher('/corr/pose', Pose, queue_size=1)


    def set_params(self):
        self.steer_scale_factor = 32.2/450  # wheel max angle / handle max angle
        self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]

        while 1: 
            if self.base_lla is not None:
                proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
                proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.base_lla[0], lon_0=self.base_lla[1], h_0=self.base_lla[2])
                self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)
                break
            else:
                pass


    def ori_cb(self, msg): # gain original position
        if self.transformer is not None and (self.corr_can_velocity is not None):
            lat = msg.lat*1e-7
            lon = msg.lon*1e-7
            x, y, _= self.transformer.transform(lon, lat, 7)
            self.ori_pos = [x, y]

            self.ori_hdg = -(msg.heading*1e-5 - 90)%360


    def loc_cb(self, msg): # gain localization data
        if self.transformer is not None and (self.corr_can_velocity is not None):
            self.noisy_pos_last = self.noisy_pos
            self.noisy_hdg_last = self.noisy_hdg

            lat = msg.lat*1e-7
            lon = msg.lon*1e-7
            x, y, _= self.transformer.transform(lon, lat, 7)
            self.noisy_pos = [x, y]

            self.noisy_hdg = -(msg.heading*1e-5 - 90)%360

            self.hAcc = msg.hAcc
            self.headAcc = msg.headAcc

            self.llh = [lat, lon]
        
            self.loc_on = True        


    def system_status_cb(self, msg):
        self.base_lla = msg.baseLLA
    

    def canoutput_cb(self, msg): # gain velocity, steering angle
        self.canoutput_update()
        self.can_velocity = float(msg.VS.data)
        self.corr_can_velocity = (self.can_velocity*3.6 \
                                  + self.params[0] + self.params[1]*(self.can_velocity*3.6) \
                                    + self.params[2]*((self.can_velocity*3.6)**2) \
                                        + self.params[3]*((self.can_velocity*3.6)**3))/3.6 # [m/s]

        handle_ang = float(msg.StrAng.data)
        self.can_steer = handle_ang*self.steer_scale_factor 
    

    def canoutput_update(self):
        self.can_velocity_last = self.can_velocity
        self.corr_can_velocity_last = self.corr_can_velocity
        self.can_steer_last = self.can_steer


    def publish(self, heading, enu):
        pos_msg = Pose()
        pos_msg.position.x = enu[0]
        pos_msg.position.y = enu[1]
        # pos_msg.position.z = not used
        pos_msg.orientation.x = self.llh[0]
        pos_msg.orientation.y = self.llh[1]
        pos_msg.orientation.z = heading
        # pos_msg.orientation.w = not used
        self.corr_pose_pub.publish(pos_msg)
