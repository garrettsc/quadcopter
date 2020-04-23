from frame import Frame
from quadcopter import Quadcopter
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')



quad = Quadcopter()
frame = Frame()

quad.THETA = np.array([[0],[np.deg2rad(-45)],[0]])


v1 = np.array([3,0,0])
v2 = np.array([-3,0,0])
v3 = np.array([0,3,0])
v4 = np.array([0,-3,0])

for idx in range(0,100):

    plt.cla()
    quad.update()
    frame.x = quad.x[0][0]
    frame.y = quad.x[1][0]
    frame.z = quad.x[2][0]
    frame._pitch = quad.pitch
    frame._roll = quad.roll
    frame._yaw = quad.yaw

    # if idx < 50:
    #     quad.W[0] = 5
    #     quad.W[1] = 5
    # else:
    #     quad.W[0] = 10
    #     quad.W[1]= 10

    for v in [v1,v2,v3,v4]:
        frame.show_vec(v,ax)

    ax.set_xlim3d(0, 15)
    ax.set_ylim3d(0,15)
    ax.set_zlim3d(0,15)
    frame.show(ax)
    print(quad.THETA)
    plt.pause(0.001)

