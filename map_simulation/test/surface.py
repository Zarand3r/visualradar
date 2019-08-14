import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random
from matplotlib import cm
# def fun(x, y):
#     return x**2 + y

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# x = y = np.arange(-3.0, 3.0, 0.05)
# X, Y = np.meshgrid(x, y)
# zs = np.array([fun(x,y) for x,y in zip(np.ravel(X), np.ravel(Y))])
# Z = zs.reshape(X.shape)

# ax.plot_surface(X, Y, Z)


def surf(x,y):
	# return x+y
	return 0

fig = plt.figure(num=1)
fig.clf()
ax = fig.add_subplot(1, 1, 1, projection='3d')

(x, y) = np.meshgrid(np.arange(-100, 100, 100), np.arange(-100, 100, 100))
z = surf(x,y)
ax.plot_surface(x, y, z, cmap=cm.copper)
ax.set(xlabel='x', ylabel='y', zlabel='z', title='test')
fig.tight_layout()
plt.show()



# fig = plt.figure(num=1)
# fig.clf()
# ax = fig.add_subplot(1, 1, 1, projection='3d')

# (x, y) = np.meshgrid(np.linspace(-1.2, 1.2, 20), 
#                      np.linspace(-1.2, 1.2, 20))
# z = np.sqrt((x-(1))**2 + (y-(-0.5))**2)

# ax.plot_surface(x, y, z, cmap=cm.magma)
# ax.set(xlabel='x', ylabel='y', zlabel='z', 
#        title='test')

# fig.tight_layout()
# plt.show()


