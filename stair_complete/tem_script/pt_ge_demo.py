# # View more python tutorials on my Youtube and Youku channel!!!
#
# # Youtube video tutorial: https://www.youtube.com/channel/UCdyjiB5H8Pu7aDTNVXTTpcg
# # Youku video tutorial: http://i.youku.com/pythontutorial
#
# # 14 - 3d
# """
# Please note, this script is for python3+.
# If you are using python2+, please modify it accordingly.
# Tutorial reference:
# http://www.python-course.eu/matplotlib_multiple_figures.php
# """
#
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib import cm
#
# fig = plt.figure()
# ax = Axes3D(fig)
# # X, Y value
# X = np.arange(0, 24, 0.1)
# Y = np.arange(0, 8, 0.1)
# X, Y = np.meshgrid(X, Y)
#
# Z = np.zeros_like(X)
# R = np.zeros_like(X)
# for i in range(1, 24, 8):
#     Z += (X > i) * 8
#     R += (X > i) * 8
#     R += Y > i
#
# R = R/R.max()
# # R = np.sqrt(X ** 2 + Y ** 2)
# # # height value
# # Z = np.sin(R)
#
#
# # ax.plot_surface(X, Y, Z, rstride=3, cstride=3, cmap=plt.get_cmap('rainbow'), facecolors=cm.Oranges(R))


import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(211, projection='3d')

for i in range(3):
    ax.plot(xs=[i, i+1], ys=[0, 0], zs=[i, i])
    ax.plot(xs=[i, i], ys=[0., 0.6], zs=[i, i])
    ax.plot(xs=[i, i+1], ys=[0.6, 0.6], zs=[i, i])
    ax.plot(xs=[i+1, i+1], ys=[0, 0.6], zs=[i, i])
    ax.plot(xs=[i, i], ys=[0, 0.6], zs=[i, i])
    ax.plot(xs=[i, i], ys=[0, 0.6], zs=[i-1, i-1])
    ax.plot(xs=[i, i], ys=[0, 0], zs=[i, i-1])
    ax.plot(xs=[i, i], ys=[0.6, 0.6], zs=[i, i-1])
# plt.show()

plt.subplot(212)
for i in range(11):
    plt.plot([0.3 * i, 0.3 * i], [0., 0.6], "--")
for i in range(5):
    plt.plot([0, 3], [0.15*i, 0.15*i], "--")

for i in range(4):
    plt.plot([1 * i, 1 * i], [0., 0.6])
for i in range(2):
    plt.plot([0, 3], [0.6*i, 0.6*i])
plt.show()

