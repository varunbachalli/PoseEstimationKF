from ReadFile import getData
from Wahba import Wahba
import matplotlib.pyplot as plt
from UtilityFunctions import DimensionalSplit, norm
import numpy as np 
from ExtendedKalmanFilter import KalmanFilter

def Comparator(q1, q2):
    return q2[0] * q2[0] + q2[1]* q1[1] + q2[2]*q1[2] + q2[3]*q1[3]
# read Data from Text File
g = getData()


# set up pure wahba 
w = Wahba(g.acc_0,g.mag_0)
WahbaQuart = []

# set up Extended KalmanFilter 
k = KalmanFilter(g.timestamp[0][0],g.mag_0,g.acc_0, 0.5)
previousT = g.timestamp[0][0]
k.setQ(1)
k.setR(0.1)
P = np.identity(4)

g.timestamp = g.timestamp[1:]
X = np.asarray([1.,0.,0.,0.])
X_k = [X]

# set up pure Gyro Quarternion

GyroQuart = [X]
p_gyro = np.identity(4)


comps = []
alpha_gyro = 0.3
Gyro = np.asarray([0.0,0.0,0.0])
for i in range(len(g.acc_1)):
    z_k, P, K_k = k.Prediction(g.gyro[i], g.timestamp[i][0], X, P)
    wahbaquart = w.getQuarternion(g.acc_1[i], g.mag_1[i],0.5,0.5)
    wahbaquart = np.asarray(g.quart_wahba[i])
    gyroquart = np.asarray(g.quart_gyro[i])
    X, P = k.Correction(g.mag_1[i], g.acc_1[i],z_k, P, K_k)
    X_k.append(X)
    GyroQuart.append(gyroquart)
    WahbaQuart.append(wahbaquart)
    previousT = g.timestamp[i][0]


Filt = DimensionalSplit(X_k)
Wahb = DimensionalSplit(WahbaQuart)
Gyr = DimensionalSplit(GyroQuart)
cpp = DimensionalSplit(g.quart_xk)

plot_these = [True, True, True, False]

plt.figure('w')
if(plot_these[0]):
    plt.plot(range(len(Filt[0])), Filt[0],label = 'Filter' ,color = 'red')
if(plot_these[1]):
    plt.plot(range(len(Wahb[0])), Wahb[0],label = 'Wahba' ,color = 'black')
if(plot_these[2]):
    plt.plot(range(len(Gyr[0])), Gyr[0],label = 'Gyro' ,color = 'green')
if(plot_these[3]):
    plt.plot(range(len(cpp[0])), cpp[0],label = 'CPP' ,color = 'blue')
plt.legend()

plt.figure('x')
if(plot_these[0]):
    plt.plot(range(len(Filt[1])), Filt[1],label = 'Filter',color = 'red')
if(plot_these[1]):
    plt.plot(range(len(Wahb[1])), Wahb[1],label = 'Wahba',color = 'black')
if(plot_these[2]):
    plt.plot(range(len(Gyr[1])), Gyr[1],label = 'Gyro' ,color = 'green')
if(plot_these[3]):
    plt.plot(range(len(cpp[1])), cpp[1],label = 'CPP' ,color = 'blue')
plt.legend()


plt.figure('y')
if(plot_these[0]):
    plt.plot(range(len(Filt[2])), Filt[2],label = 'Filter',color = 'red')
if(plot_these[1]):
    plt.plot(range(len(Wahb[2])), Wahb[2],label = 'Wahba',color = 'black')
if(plot_these[2]):
    plt.plot(range(len(Gyr[2])), Gyr[2],label = 'Gyro' ,color = 'green')
if(plot_these[3]):
    plt.plot(range(len(cpp[2])), cpp[2],label = 'CPP' ,color = 'blue')

plt.legend()
plt.figure('z')
if(plot_these[0]):
    plt.plot(range(len(Filt[3])), Filt[3],label = 'Filter',color = 'red')
if(plot_these[1]):
    plt.plot(range(len(Wahb[3])), Wahb[3],label = 'Wahba',color = 'black')
if(plot_these[2]):
    plt.plot(range(len(Gyr[3])), Gyr[3],label = 'Gyro' ,color = 'green')
if(plot_these[3]):
    plt.plot(range(len(cpp[3])), cpp[3],label = 'CPP' ,color = 'blue')
plt.legend()
plt.show()
