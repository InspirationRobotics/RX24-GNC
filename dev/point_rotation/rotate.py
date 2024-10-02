import numpy as np
import time

def rotationMatrix(a):
    return np.array([
        [np.cos(a), -np.sin(a), 0],
        [np.sin(a), np.cos(a), 0],
        [0, 0, 1]
    ])

def rotatePointCloud(xyz, degrees, x0, y0, z0):
    return xyz @ rotationMatrix(degrees * np.pi / 180) + [x0, y0, z0]


x0 = 10
y0 = 10
z0 = 0

degrees = 57

startest = time.time()
xyz = np.random.randint(0, 100, (10000, 3))
for i in range(100):
    start = time.time()
    newxyz = rotatePointCloud(xyz, degrees, x0, y0, z0)
    print(time.time() - start, end='\r')

print()
print(time.time() - startest)