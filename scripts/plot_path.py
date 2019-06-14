import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

pose_pre_file = open("pose_pre.txt").readlines()
pose_post_file = open("pose_post.txt").readlines()
xs, ys, zs = [], [], []
xe, ye, ze = [], [], []
for i, ele in enumerate(pose_pre_file):
    if i%3 == 0:
        x = float(ele)
        xs.append(x)
    if i%3 == 1:
        y = float(ele)
        ys.append(y)
    if i%3 == 2:
        z = float(ele)
        zs.append(z)
    

for i, ele in enumerate(pose_post_file):
    if i%3 == 0:
        x = float(ele)
        xe.append(x)
    if i%3 == 1:
        y = float(ele)
        ye.append(y)
    if i%3 == 2:
        z = float(ele)
        ze.append(z)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax.plot(xs, ys, zs, "b-", lw=3, alpha=0.8)
# ax.scatter(xs[index], ys[index], zs[index], color = 'red')
# ax.scatter(xs, ys, zs, color = 'blue')
# ax.plot(xe, ye, ze, "r-", lw=3, alpha=0.8)
# ax.scatter(xe, ye, ze, color = 'red')
ax.scatter(np.array(xe)-np.array(xs), np.array(ye)-np.array(ys), \
    np.array(ze)-np.array(zs), color = 'red')
ax.axis('equal')
plt.show()