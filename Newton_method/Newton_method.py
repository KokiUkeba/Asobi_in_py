#!/usr/bin/python3
import numpy as np
from matplotlib import pyplot as plt

def f(x):
    return x**3 - 16*x

def df(x):
    return 3*x**2 - 16
    
def Newton_method(x, d):
    while(f(x) > d):
        new_x = x - f(x)/df(x)
        x = new_x
    return x

zeros = np.zeros(100)
x = np.linspace(-5, 5, 100)
y = np.array(f(x[:]))

print(Newton_method(5, 0.001))

plt.plot(x, zeros, '--', color='k')
plt.plot(x, y)

plt.show()
