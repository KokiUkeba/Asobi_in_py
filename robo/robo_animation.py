#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

class PID:
    P_list = [0.]
    I_list = [0.]
    D_list = [0.]
    output_list = [0.]
    def __init__(self, Kp, Ki, Kd, start=0., goal=90.):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.start = start
        self.goal = goal
        self.P_component = 0.
        self.I_component = 0.
        self.D_component = 0.

    def controller(self, current_data, pre_data=0):
        self.P_component = self.Kp * (self.goal - current_data)
        self.I_component += self.Ki * (self.goal - current_data)
        self.D_component = self.Kd * (current_data - pre_data)
        m = self.P_component + self.I_component - self.D_component
        self.P_list.append(self.P_component)
        self.I_list.append(self.I_component)
        self.D_list.append(self.D_component)
        self.output_list.append(m)
        return m

robo_weight = 5.
robo_place = []
robo_velocity = []
robo_acceleration = []
robo_angle = []
robo_vector = []
robo_turning_radius = 100.
robo_wheele_1 = []
robo_wheele_2 = []
robo_wheele_3 = []
robo_wheele_4 = []

Start = [0., 0.]
Goal = [6, 9]
Goal_angle = 120.

Time_length = 20.
Update_interval = 0.001

Start_Goal = math.sqrt((Goal[0]-Start[0])**2 + (Goal[1]-Start[1])**2)
global_theta = math.atan(Goal[1]/Goal[0])

pid1 = PID(0.0005, 0.000000001, 0.00005, 0., Start_Goal)
pid_angle = PID(0.0005, 0.000000001, 0.0005, 0., Goal_angle%360)

time_list = np.linspace(0., Time_length, int(Time_length/Update_interval))

r_list = [0.]
robo_place.append(Start)
robo_angle.append(0.)
robo_vector.append([robo_place[0][0]+math.cos(math.radians(robo_angle[0])), robo_place[0][1]+math.sin(math.radians(robo_angle[0]))])
robo_wheele_1.append([robo_place[0][0]+math.cos(math.radians(45 + robo_angle[0])), robo_place[0][1]+math.sin(math.radians(45 + robo_angle[0]))])
robo_wheele_2.append([robo_place[0][0]+math.cos(math.radians(135 + robo_angle[0])), robo_place[0][1]+math.sin(math.radians(135 + robo_angle[0]))])
robo_wheele_3.append([robo_place[0][0]+math.cos(math.radians(225 + robo_angle[0])), robo_place[0][1]+math.sin(math.radians(225 + robo_angle[0]))])
robo_wheele_4.append([robo_place[0][0]+math.cos(math.radians(315 + robo_angle[0])), robo_place[0][1]+math.sin(math.radians(315 + robo_angle[0]))])
robo_velocity.append(0.)
robo_acceleration.append(0.)
for i in range(len(time_list)-1):
    try:
        m = pid1.controller(r_list[-1], r_list[-2])
        a = pid_angle.controller(robo_angle[-1], robo_angle[-2])
    except:
        m = pid1.controller(r_list[-1])
        a = pid_angle.controller(robo_angle[-1])
    r_list.append(r_list[-1] + m)
    robo_place.append([r_list[-1]*math.cos(global_theta), r_list[-1]*math.sin(global_theta)])
    robo_angle.append(robo_angle[-1] + a)
    robo_vector.append([robo_place[i+1][0]+math.cos(math.radians(robo_angle[-1])), robo_place[i+1][1]+math.sin(math.radians(robo_angle[-1]))])
    robo_wheele_1.append([robo_place[i+1][0]+math.cos(math.radians(45 + robo_angle[-1])), robo_place[i+1][1]+math.sin(math.radians(45 + robo_angle[-1]))])
    robo_wheele_2.append([robo_place[i+1][0]+math.cos(math.radians(135 + robo_angle[-1])), robo_place[i+1][1]+math.sin(math.radians(135 + robo_angle[-1]))])
    robo_wheele_3.append([robo_place[i+1][0]+math.cos(math.radians(225 + robo_angle[-1])), robo_place[i+1][1]+math.sin(math.radians(225 + robo_angle[-1]))])
    robo_wheele_4.append([robo_place[i+1][0]+math.cos(math.radians(315 + robo_angle[-1])), robo_place[i+1][1]+math.sin(math.radians(315 + robo_angle[-1]))])
    robo_velocity.append((r_list[-1] - r_list[-2])/Update_interval)
    robo_acceleration.append((robo_velocity[-1] - robo_velocity[-2])/Update_interval)

robo_place_np = np.array(robo_place)
robo_vector_np = np.array(robo_vector)
robo_wheele_1_np = np.array(robo_wheele_1)
robo_wheele_2_np = np.array(robo_wheele_2)
robo_wheele_3_np = np.array(robo_wheele_3)
robo_wheele_4_np = np.array(robo_wheele_4)

#グラフ描画用の配列をスライス
step = 200
time_list = time_list[::step]
r_list = r_list[::step]
robo_place_np = robo_place_np[::step]
robo_angle = robo_angle[::step]
robo_wheele_1_np = robo_wheele_1_np[::step]
robo_wheele_2_np = robo_wheele_2_np[::step]
robo_wheele_3_np = robo_wheele_3_np[::step]
robo_wheele_4_np = robo_wheele_4_np[::step]

fig, ax = plt.subplots(2, 2, squeeze=False)

def update(f):
    fig.suptitle('frame{}/{}'.format(f, len(time_list)))
    ax[0, 0].cla()
    ax[0, 0].plot(robo_place_np[:, 0], robo_place_np[:, 1], color='gray')
    ax[0, 0].plot(robo_wheele_1_np[:, 0], robo_wheele_1_np[:, 1], color='pink')
    ax[0, 0].plot(robo_wheele_2_np[:, 0], robo_wheele_2_np[:, 1], color='navajowhite')
    ax[0, 0].plot(robo_wheele_3_np[:, 0], robo_wheele_3_np[:, 1], color='lightgreen')
    ax[0, 0].plot(robo_wheele_4_np[:, 0], robo_wheele_4_np[:, 1], color='lightskyblue')
    ax[0, 0].plot(robo_place_np[f, 0], robo_place_np[f, 1], 'o', color='black')
    ax[0, 0].plot(robo_wheele_1_np[f, 0], robo_wheele_1_np[f, 1], 'o', color='red', label='1')
    ax[0, 0].plot(robo_wheele_2_np[f, 0], robo_wheele_2_np[f, 1], 'o', color='orange', label='2')
    ax[0, 0].plot(robo_wheele_3_np[f, 0], robo_wheele_3_np[f, 1], 'o', color='green', label='3')
    ax[0, 0].plot(robo_wheele_4_np[f, 0], robo_wheele_4_np[f, 1], 'o', color='blue', label='4')
    ax[0, 0].quiver(robo_place_np[f, 0], robo_place_np[f, 1], math.cos(math.radians(robo_angle[f])), math.sin(math.radians(robo_angle[f])), angles='xy', scale_units='xy', scale=2, units='xy', width=0.1)
    ax[0, 0].set_title("goal=[{},{}], goal_angle={}".format(Goal[0], Goal[1], Goal_angle))
    ax[0, 0].set_aspect('equal', 'datalim')

    ax[0, 1].cla()
    ax[0, 1].axhline(y=Start_Goal, xmin=0, xmax=1, color='blue', linestyle='dashed')
    ax[0, 1].plot(time_list, r_list, color='gray')
    ax[0, 1].plot(time_list[f], r_list[f], 'o', color='black')
    ax[0, 1].set_title("Kp={}, Ki={}, Kd={}".format(pid1.Kp, pid1.Ki, pid1.Kd))

    #ax[1, 0].cla()
    #ax[1, 0].plot(time_list, robo_velocity, color='gray')
    #ax[1, 0].plot(time_list[f], robo_velocity[f], 'o', color='black')

    ax[1, 1].cla()
    ax[1, 1].axhline(y=Goal_angle, xmin=0, xmax=1, color='blue', linestyle='dashed')
    ax[1, 1].plot(time_list, robo_angle, color='gray')
    ax[1, 1].plot(time_list[f], robo_angle[f], 'o', color='black')
    ax[1, 1].set_title("Kp={}, Ki={}, Kd={}".format(pid_angle.Kp, pid_angle.Ki, pid_angle.Kd))


anim = FuncAnimation(fig, update, frames=len(time_list), interval = 0.001)
plt.show()

