#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt

class PID:
    P_list = [0.]
    I_list = [0.]
    D_list = [0.]
    output_list = [0.]
    def __init__(self, Kp, Ki, Kd, start=0., goal=90.):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Time_length = Time_length
        self.start = start
        self.goal = goal
        self.Update_interval = Update_interval
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
robo_turning_radius = 100.

Start = [0., 0.]
Goal = [4, 3]

Time_length = 20.
Update_interval = 0.001

Start_Goal = math.sqrt((Goal[0]-Start[0])**2 + (Goal[1]-Start[1])**2)
global_theta = math.atan(Goal[1]/Goal[0])

pid1 = PID(0.00035, 0., 0., 0., Start_Goal)

time_list = np.linspace(0., Time_length, int(Time_length/Update_interval))

r_list = [0.]
robo_place.append(Start)
robo_velocity.append(0.)
for i in range(len(time_list)-1):
    try:
        m = pid1.controller(r_list[-1], r_list[-2])
    except:
        m = pid1.controller(r_list[-1])
    r_list.append(r_list[-1] + m)
    robo_place.append([r_list[-1]*math.cos(global_theta), r_list[-1]*math.sin(global_theta)])
    robo_velocity.append((r_list[-1] - r_list[-2])/Update_interval)

robo_place_np = np.array(robo_place)

fig, ax = plt.subplots(1, 3, squeeze=False)
ax[0, 0].plot(robo_place_np[:, 0], robo_place_np[:, 1], '.')
ax[0, 1].plot(time_list, robo_velocity, '.')
ax[0, 2].plot(time_list, r_list, '.')
plt.show()
