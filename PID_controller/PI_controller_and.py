#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt

G = 9.81
Kp = 0.01 
Ki = 0.008
Kd = 0.0
Time_length = 3.
Update_interval = 0.001
Theta_start = 0.0
Theta_goal = 90.0
Ball_weight = 0.5
Arm_length = 0.3
I_component = 0.
P_list = [0]
I_list = [0]
D_list = [0]

def Ball_vertical_component(theta_current):
    return Ball_weight * G * Arm_length * math.sin(math.radians(theta_current % 360))

def PID_controller(theta_current, I_component):
    P_component = Kp * (Theta_goal - theta_current)
    I_component += Ki * (Theta_goal - theta_current)
    try:
        D_component = Kd * (theta_current - theta_list[-2])
    except:
        D_component = 0
    m = P_component + I_component - D_component
    P_list.append(P_component)
    I_list.append(I_component)
    D_list.append(D_component)
    return m, I_component

time_list = np.linspace(0, Time_length, int(Time_length/Update_interval))

goal_list = [Theta_goal for i in time_list]

theta_current = Theta_start
theta_list = [theta_current]
for i in range(len(time_list)-1):
    m, I_component = PID_controller(theta_current, I_component)
    output = m - Ball_vertical_component(theta_current)
    theta_list.append(theta_current + output)
    theta_current = theta_list[-1]

plt.plot(time_list, goal_list, '--', color='k')
plt.plot(time_list, theta_list)

plt.show()
