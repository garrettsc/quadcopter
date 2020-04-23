import matplotlib.pyplot as plt
import numpy as np


def power(T,rho, r):

    A = 2*np.pi*r
    return np.sqrt(T**3 / (2*rho*A))



thrust = np.arange(0,2000)
rho=1.225
r = np.linspace(.1,1,thrust.shape[0])

p = power(thrust,rho,r)

plt.scatter(p,thrust)
plt.ylabel('Thrust [newtons]')
plt.xlabel('Power [watts]')
plt.show()