#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt

g = 9.81
Time_length = 100.
Update_interval = 0.001
Start = 0.
Goal = 90.
Ball_weight = 0.5
Arm_length = 0.3

class PID:
    P_list = [0.]
    I_list = [0.]
    D_list = [0.]
    output_list = [0.]
    def __init__(self, Kp, Ki, Kd, Time_length=10., start=0., goal=90., Update_interval=0.001):
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
        m = self.P_component + self.I_component + self.D_component
        self.P_list.append(self.P_component)
        self.I_list.append(self.I_component)
        self.D_list.append(self.D_component)
        self.output_list.append(m)
        return m

def Ball_vertical_component(theta_current):
    return Ball_weight * g * Arm_length * math.sin(math.radians(theta_current % 360))

pid1 = PID(0.002, 0.000009, 0.5, Time_length, Start, Goal, Update_interval)
pid2 = PID(0.002, 0.000009, 0., Time_length, Start, Goal, Update_interval)

time_list = np.linspace(0, Time_length, int(Time_length/Update_interval))
goal_list = [Goal for i in time_list]

theta_list_1 = [0.]
theta_list_2 = [0.]
for i in range(len(time_list)-1):
    try:
        m1 = pid1.controller(theta_list_1[-1], theta_list_1[-2])
        m2 = pid2.controller(theta_list_2[-1], theta_list_2[-2])
    except:
        m1 = pid1.controller(theta_list_1[-1])
        m2 = pid2.controller(theta_list_2[-1])
    output1 = m1 - Ball_vertical_component(theta_list_1[-1])
    output2 = m2 - Ball_vertical_component(theta_list_2[-1])
    theta_list_1.append(theta_list_1[-1] + output1)
    theta_list_2.append(theta_list_2[-1] + output2)
    #time_lengthの半分の時間が経過したときボールの重さを2倍にする
    if i == int(len(time_list)/2):
        Ball_weight *= 2

plt.plot(time_list, goal_list, '--', color='k')
plt.plot(time_list, theta_list_1, label='1')
plt.plot(time_list, theta_list_2, label='2')

plt.legend()
plt.show()
