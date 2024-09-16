#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt

P0 = np.array([2, 2])
P1 = np.array([5, 2])
P2 = np.array([8, 4])
P3 = np.array([9, 9])
P4 = np.array([12, 8])

controll_dot = np.array([P0, P1, P2, P3, P4])

t = np.linspace(0, 1, 100)
bezier_list = []

for i in t:
    dot = (1 - i)**4 *P0 + 4*i*(1 - i)**3 *P1 +6*i**2 *(1 - i)**2 *P2 + 4*i**3 *(1 - i)*P3 + i**4 *P4
    dot_list = dot.tolist()
    bezier_list.append(dot_list)

bezier_nparray = np.array(bezier_list)

plt.plot(controll_dot[:, 0], controll_dot[:, 1], 'o')
plt.plot(bezier_nparray[:, 0], bezier_nparray[:, 1], '.')
plt.show()
