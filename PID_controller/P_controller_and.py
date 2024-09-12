#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt

G = 9.81
Kp = 9
Update_interval = 0.001
Theta_start = 0.0
Theta_goal = 90.0
Time_length = 10.
Ball_weight = 0.1
Arm_length = 0.3

def Ball_vertical_conponent(theta_current):
    return Ball_weight * G * Arm_length * math.sin(theta_current)

def P_controller(theta_current):
    m = Kp*(Theta_goal - theta_current) * Update_interval
    return m if Ball_vertical_conponent(theta_current) < m else 0

time_list = np.linspace(0, Time_length, int(Time_length/Update_interval))

goal_list = [Theta_goal for i in time_list]

theta_current = Theta_start
theta_list=[theta_current]
for i in range(len(time_list)-1):
    theta_list.append(theta_current + P_controller(theta_current))
    theta_current = theta_list[-1]

plt.plot(time_list, goal_list, '--', color='k')
plt.plot(time_list, theta_list)

plt.show()
print(theta_list)
