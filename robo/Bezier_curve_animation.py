#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

controll_dot = np.array([[2, 2], [5, 2], [5, 5], [7, 5]])

t = np.linspace(0, 1, 100)
bezier_list = []

for i in t:
    dot = np.array([0., 0.])
    for j in range(len(controll_dot)):
        dot += (math.factorial(len(controll_dot)-1) // math.factorial(j) // math.factorial(len(controll_dot) - (j+1))) * (1 - i)**(len(controll_dot) - (j+1)) *i**(j) * controll_dot[j][:]
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
