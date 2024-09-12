#!/usr/bin/python3
import numpy as np
from matplotlib import pyplot as plt

T = 0.001
X_0 = 0.
GOAL = 20.
Kp = 2
Ki = 2

def P_controller(x):
    return Kp*(GOAL - x)*T

def I_controller(x):
    return Ki*(GOAL - x)*T

t = np.linspace(0, 10, 10000)

X_goal = [GOAL for i in t]

X = [X_0]
x = X_0
for i in range(len(t) - 1):
    X.append(x + P_controller(x) + I_controller(x))
    x = X[-1]

plt.plot(t, X_goal, '--', color='k')
plt.plot(t, X)

plt.xlabel('time')
plt.ylabel('X')

#plt.legend()
plt.show()
