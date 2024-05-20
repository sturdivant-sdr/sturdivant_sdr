
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("acq_plane.csv", delimiter=',')

chips = np.arange(1,20001,1)
freqs = np.arange(-5000,5200,200)

x,y = np.meshgrid(freqs, chips)
fig = plt.figure(figsize =(14, 9))
ax = plt.axes(projection ='3d')
ax.plot_surface(x, y, data, rstride=10, cstride=1, cmap=plt.get_cmap('turbo'))

plt.show()

print(data.shape)
print(data)