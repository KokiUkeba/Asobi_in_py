#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

P0 = np.array([2, 2])
P1 = np.array([5, 2])
P2 = np.array([8, 4])
P3 = np.array([3, 3])
P4 = np.array([2, 2])
controll_dot = np.array([P0, P1, P2, P3, P4])

t = np.linspace(0, 1, 100)
bezier_list = []

for i in t:
    dot =(1 - i)**4 * P0 + 4 * (1 - i)**3 * i * P1 + 6 * (1 - i)**2 * i**2 * P2 + 4  * (1 - i) * i**3 * P3 + i**4 * P4
    dot_list = dot.tolist()
    bezier_list.append(dot_list)

bezier_nparray = np.array(bezier_list)

fig, ax = plt.subplots(1, 1, squeeze=False)

def update(f):
    ax[0, 0].cla()
    ax[0, 0].plot(bezier_nparray[f, 0], bezier_nparray[f, 1], 'o', color='red')
    ax[0, 0].plot(bezier_nparray[:, 0], bezier_nparray[:, 1], color='pink')
    ax[0, 0].plot(controll_dot[:, 0], controll_dot[:, 1], '-.', linewidth=0.5)
    ax[0, 0].set_aspect('equal', 'datalim')

anim = FuncAnimation(fig, update, frames=len(bezier_nparray), interval=1)
plt.show()
