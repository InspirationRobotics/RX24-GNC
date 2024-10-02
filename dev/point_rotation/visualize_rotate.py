import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import time

def rotateMatrix(a):
    return np.array([
        [np.cos(a), -np.sin(a), 0],
        [np.sin(a), np.cos(a), 0],
        [0, 0, 1]
    ])

xyz = np.random.randint(0, 100, (200, 3))

xyz[0] = [0, 0, 0]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2])
ax.set_xlim(-30, 130)
ax.set_ylim(-50, 110)
ax.set_zlim(-50, 150)

plotdata = ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2])

x0 = 20
y0 = 0
z0 = 0

def anim(i):
    newxyz = xyz @ rotateMatrix(i * np.pi / 180) + [x0, y0, z0]
    plotdata._offsets3d = (newxyz[:, 0], newxyz[:, 1], newxyz[:, 2])
    return [plotdata]

theAnim = animation.FuncAnimation(fig, anim, interval=40, blit=False, frames=360, repeat=True)
plt.show()
