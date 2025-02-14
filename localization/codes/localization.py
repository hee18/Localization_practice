import math
import rospy
import matplotlib.pyplot as plt

import time
import sys
import signal

from ros_handler import ROSHandler

ori_poss = []
ori_hdgs = []
noisy_poss = []
noisy_hdgs = []
corr_poss = []
corr_hdgs = []

class Localization:
    def __init__(self):
        self.RH = ROSHandler()
        self.initiated = False

        self.noisy_pos_valid = True
        self.noisy_hdg_valid = True

        self.corr_pos = None
        self.corr_hdg = None

        self.dr_pos = None
        self.dr_hdg = None

        self.dr_pos_cnt = 0
        self.dr_hdg_cnt = 0

        self.restart_timer = rospy.Time.now()

        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.ori_pos_plot, = self.ax.plot([], [], 'b-', label='Original', linewidth=2)  
        self.noisy_pos_plot, = self.ax.plot([], [], 'r-.', label='Noisy')  
        self.corr_pos_plot, = self.ax.plot([], [], 'g--', label='Corrected')  
        self.ax.legend()
        self.ax.grid(True)

        self.max_pos_error = 0
        self.max_hdg_error = 0


    def dr_timer(self, sec):
        if time.time() - self.dr_time > sec:
            self.dr_time = time.time()
            return True
        else:
            return False
        
            
    def initiate(self):
        if not self.initiated:
            while not self.init_all_msgs():
                self.update_sensor_data()
                self.corr_pos = self.RH.noisy_pos
                self.corr_hdg = self.RH.noisy_hdg
            self.dr_time = time.time()
            self.initiated = True


    def init_all_msgs(self):
        key1, key2 = False, False
        vehicle_datas = [self.RH.corr_can_velocity_last, self.RH.corr_can_velocity, self.RH.can_velocity_last, self.RH.can_velocity, self.RH.can_steer, self.RH.can_steer_last]
        loc_datas = [self.corr_hdg, self.RH.noisy_hdg, self.RH.noisy_pos_last[0], self.RH.noisy_pos[0], self.RH.noisy_pos_last[1], self.RH.noisy_pos[1]]
        if not None in vehicle_datas:
            key1 = True
        if not None in loc_datas:
            key2 = True

        if key1 and key2:
            return True

        return False


    def update_sensor_data(self): # updates when localization data callbacked
        while self.RH.loc_on:
            self.RH.can_velocity_last = self.RH.can_velocity
            self.RH.corr_can_velocity_last = self.RH.corr_can_velocity
            self.RH.noisy_hdg_last = self.RH.noisy_hdg
            self.RH.noisy_pos_last = self.RH.noisy_pos
            self.RH.loc_on = False


    def calculate_dr_pos(self):
        dt = 0.05

        x_delta = (dt * self.RH.corr_can_velocity_last) * math.cos(math.radians(self.corr_hdg))
        y_delta = (dt * self.RH.corr_can_velocity_last) * math.sin(math.radians(self.corr_hdg))
        self.dr_pos = [self.corr_pos[0] + x_delta, self.corr_pos[1] + y_delta]

    
    def calculate_dr_hdg(self):
        dt = 0.05

        rad_delta = math.radians(self.RH.can_steer_last + 0.047)  # 0.047 => wheelangle zero offset
        self.dr_hdg = self.corr_hdg + (dt * math.degrees((self.RH.corr_can_velocity_last / 2.72) * math.tan(rad_delta)))  # 2.72 => wheelbase


    def timer(self, sec):   
        if (rospy.Time.now() - self.restart_timer).to_sec() >= sec:
            result = True
        else:
            result = False

        return result
    

    def calculate_pos_error(self):
        xerror = self.corr_pos[0] - self.RH.ori_pos[0]
        yerror = self.corr_pos[1] - self.RH.ori_pos[1]
        
        pos_error = (xerror**2+yerror**2)**0.5

        if pos_error > self.max_pos_error:
            self.max_pos_error = pos_error
        
    
    def calculate_hdg_error(self):
        hdg_val = abs(self.corr_hdg - self.RH.ori_hdg)
        hdg_error = min(hdg_val, 360-hdg_val)
        
        if hdg_error > self.max_hdg_error:
            self.max_hdg_error = hdg_error


    def update_corr_pos(self):
        if not None in [self.corr_pos, self.RH.noisy_pos]:
            noisy_diff = ((self.RH.noisy_pos[0]-self.corr_pos[0])**2 + (self.RH.noisy_pos[1]-self.corr_pos[1])**2)**0.5
        if noisy_diff < 5:
            self.noisy_pos_valid = True
        elif noisy_diff >= 5:
            self.noisy_pos_valid = False
        
        if None in [self.corr_pos, self.dr_pos]:
            dr_pos_valid = False
        
        dr_diff = ((self.dr_pos[0]-self.corr_pos[0])**2 + (self.dr_pos[1]-self.corr_pos[1])**2)**0.5
        if dr_diff < 5:
            dr_pos_valid = True
        else:
            dr_pos_valid = False
        
        if self.noisy_pos_valid:
            self.corr_pos = self.RH.noisy_pos
        elif dr_pos_valid:
            self.corr_pos = self.dr_pos
            self.dr_pos_cnt += 1
            print(f"DR_POS has used total {self.dr_pos_cnt * 0.05:.2f} sec!")


    def update_corr_hdg(self):
        noisy_val = abs(self.dr_hdg - self.RH.noisy_hdg)
        noisy_diff = min(noisy_val, 360-noisy_val)
        if (noisy_diff < 5 and self.RH.headAcc < 50000) or self.RH.corr_can_velocity*3.6 < 10:
            self.noisy_hdg_valid = True
        elif noisy_diff >= 5 or self.RH.headAcc >= 50000:
            self.noisy_hdg_valid = False
        
        if None in [self.corr_hdg, self.dr_hdg]:
            dr_hdg_valid = False

        dr_val = abs(self.corr_hdg - self.dr_hdg)
        dr_diff = min(dr_val, 360 - dr_val)
        if dr_diff < 5:
            dr_hdg_valid = True
        else:
            dr_hdg_valid = False
        
        if self.noisy_hdg_valid:
            self.corr_hdg = self.RH.noisy_hdg
        elif dr_hdg_valid:
            self.corr_hdg = self.dr_hdg
            self.dr_hdg_cnt += 1
            print(f"DR_HDG has used total {self.dr_hdg_cnt * 0.05:.2f} sec!")

        
    def update_plot(self, target):
        # POS graph update
        if target == "POS" and self.corr_pos is not None:
            self.ax.clear()
            self.ax.set_xlim(-200, 1200)
            self.ax.set_ylim(-1200, 300)
            self.ax.set_xlabel("X Position (m)")
            self.ax.set_ylabel("Y Position (m)")
            self.ax.grid(True)

            ori_poss.append(self.RH.ori_pos)
            self.ax.plot([pos[0] for pos in ori_poss], [pos[1] for pos in ori_poss], 'b-', label='Original Pos Trajectory', linewidth=2)

            noisy_poss.append(self.RH.noisy_pos)
            self.ax.plot([pos[0] for pos in noisy_poss], [pos[1] for pos in noisy_poss], 'r-.', label='Noisy Pos Trajectory')

            corr_poss.append(self.corr_pos)
            self.ax.plot([pos[0] for pos in corr_poss], [pos[1] for pos in corr_poss], 'g--', label='Corrected Pos Trajectory')

            self.ax.legend()
            self.fig.canvas.draw()
            plt.pause(0.01)

        # HDG graph update
        elif target == "HDG" and self.corr_hdg is not None:
            self.ax.clear()
            self.ax.set_ylim(-10, 400)
            self.ax.set_xlabel("Frame Step")
            self.ax.set_ylabel("Heading Angle (deg)")
            self.ax.grid(True)

            ori_hdgs.append(self.RH.ori_hdg)
            self.ax.plot(ori_hdgs, 'b-', label='Original Hdg Trajectory', linewidth=2)

            noisy_hdgs.append(self.RH.noisy_hdg)
            self.ax.plot(noisy_hdgs, 'r-.', label='Noisy Hdg Trajectory')

            corr_hdgs.append(self.corr_hdg)
            self.ax.plot(corr_hdgs, 'g--', label='Corrected Hdg Trajectory')

            self.ax.legend()
            self.fig.canvas.draw()
            plt.pause(0.01)


    def run(self): 
        
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.initiate() # initiate module
            self.update_sensor_data() # update CAN, Localization sensor datas 
            self.calculate_dr_pos() # estimate current pos from last pos & sensor datas
            self.calculate_dr_hdg() # estimate current hdg from last hdg & sensor datas
            self.update_corr_pos() # update corr_pos variable
            self.update_corr_hdg() # update corr_hdg variable
            self.calculate_pos_error() # calculate pos error
            self.calculate_hdg_error() # calculate hdg error
            # self.update_plot(target="POS") # plot (choose: POS, HDG)
            self.update_plot(target="HDG") # plot (choose: POS, HDG)

            if self.corr_hdg is not None and self.corr_pos is not None:
                self.RH.publish(self.corr_hdg, self.corr_pos)
            
            rate.sleep()


def signal_handler(sig, frame):
    print(f"Max POS Error: {localization.max_pos_error:.2f} m, Max HDG Error: {localization.max_hdg_error:.2f} deg")
    sys.exit(0)

def main():
    global localization
    signal.signal(signal.SIGINT, signal_handler)
    localization = Localization()
    localization.run()

if __name__ == "__main__":
    main()