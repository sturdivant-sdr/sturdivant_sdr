
import numpy as np
import matplotlib.pyplot as plt

# data_lf = np.loadtxt("tracking_lf.csv", delimiter=',', skiprows=1)
# data_kf = np.loadtxt("tracking_kf.csv", delimiter=',', skiprows=1)
data = np.loadtxt("tracking_ch1.csv", delimiter=',', skiprows=1)

# fig1 = plt.figure(figsize =(14, 9))
# plt.plot(data[50000:,0], label='IP')
# plt.plot(data[50000:,1], label='IE')
# plt.plot(data[50000:,2], label='IL')
# plt.legend()

fig2 = plt.figure(figsize =(14, 9))
plt.plot(data[:,0], data[:,1], label='IP')
plt.plot(data[:,0], data[:,4], label='QP')
plt.legend()

# fig3 = plt.figure(figsize =(14, 9))
# plt.plot(data_lf[:,6], label='carrier doppler lf')
# plt.plot(data_kf[:,6], label='carrier doppler kf')
# plt.legend()

fig3 = plt.figure(figsize =(14, 9))
plt.plot(data[:,0], data[:,7], label='carrier doppler')
plt.legend()

plt.show()