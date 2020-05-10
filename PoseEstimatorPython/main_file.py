from ReadFile import getData
from Wahba import Wahba
import matplotlib.pyplot as plt
from UtilityFunctions import DimensionalSplit
import numpy as np 
from ExtendedKalmanFilter import KalmanFilter

# read Data from Text File
g = getData()


# set up pure wahba 
w = Wahba(g.acc_0,g.mag_0)
WahbaQuart = []

# set up Extended KalmanFilter 
k = KalmanFilter(g.timestamp[0][0],g.mag_0,g.acc_0, 0.5)
previousT = g.timestamp[0][0]
k.setQ(0.01)
k.setR(0.1)
P = np.identity(4)

g.timestamp = g.timestamp[1:]
X = np.asarray([1.,0.,0.,0.])
X_k = [X]

# set up pure Gyro Quarternion

GyroQuart = [X]
p_gyro = np.identity(4)



for i in range(len(g.acc_1)):
    z_k, P, K_k = k.Prediction(g.gyro[i], g.timestamp[i][0], X, P)
    X, P = k.Correction(g.mag_1[i], g.acc_1[i],z_k, P, K_k)
    WahbaQuart.append(w.getQuarternion(g.acc_1[i], g.mag_1[i],0.5,0.5))
    X_k.append(X)
    GyroQuart.append(KalmanFilter.RungeKutta4(GyroQuart[-1],g.timestamp[i][0]- previousT,g.gyro[i]))
    previousT = g.timestamp[i][0]


Filt = DimensionalSplit(X_k)
Wahb = DimensionalSplit(WahbaQuart)
Gyr = DimensionalSplit(GyroQuart)
cpp = DimensionalSplit(g.quart_xk)


plt.figure('w')
plt.plot(range(len(Filt[0])), Filt[0],label = 'Filter' ,color = 'red')
# plt.plot(range(len(Wahb[0])), Wahb[0],label = 'Wahba' ,color = 'black')
plt.plot(range(len(Gyr[0])), Gyr[0],label = 'Gyro' ,color = 'green')
# plt.plot(range(len(cpp[0])), cpp[0],label = 'CPP' ,color = 'blue')
plt.legend()

plt.figure('x')
plt.plot(range(len(Filt[1])), Filt[1],label = 'Filter',color = 'red')
# plt.plot(range(len(Wahb[1])), Wahb[1],label = 'Wahba',color = 'black')
plt.plot(range(len(Gyr[1])), Gyr[1],label = 'Gyro' ,color = 'green')
# plt.plot(range(len(cpp[1])), cpp[1],label = 'CPP' ,color = 'blue')
plt.legend()
plt.figure('y')
plt.plot(range(len(Filt[2])), Filt[2],label = 'Filter',color = 'red')
# plt.plot(range(len(Wahb[2])), Wahb[2],label = 'Wahba',color = 'black')
plt.plot(range(len(Gyr[2])), Gyr[2],label = 'Gyro' ,color = 'green')
# plt.plot(range(len(cpp[2])), cpp[2],label = 'CPP' ,color = 'blue')
plt.legend()
plt.figure('z')
plt.plot(range(len(Filt[3])), Filt[3],label = 'Filter',color = 'red')
# plt.plot(range(len(Wahb[3])), Wahb[3],label = 'Wahba',color = 'black')
plt.plot(range(len(Gyr[3])), Gyr[3],label = 'Gyro' ,color = 'green')
# plt.plot(range(len(cpp[3])), cpp[3],label = 'CPP' ,color = 'blue')
plt.legend()
plt.show()
