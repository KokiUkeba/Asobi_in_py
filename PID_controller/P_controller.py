#!/usr/bin/python3
import numpy as np
from matplotlib import pyplot as plt

T = 0.001
X_0 = 0.
GOAL = 20.
Kp = 2

def P_controller(x):
    return Kp*(GOAL - x)*T

t = np.linspace(0, 10, 10000)

y_goal = [GOAL for i in t]

y = [X_0]
x = X_0
for i in range(len(t) - 1):
    y.append(x + P_controller(x))
    x = y[-1]

plt.plot(t, y_goal, '--', color='k')
plt.plot(t, y)

plt.show()
