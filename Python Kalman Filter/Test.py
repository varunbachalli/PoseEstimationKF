from ReadFile import getData
import matplotlib.pyplot as plt 
import numpy as np
from UtilityFunctions import DimensionalSplit
g = getData()

Gyro = np.asarray([0.0,0.0,0.0])
Accel = np.asarray([0.0,0.0,0.0])
Magnet = np.asarray([0.0,0.0,0.0])
alpha_gyro = 1.0
alpha_magn = 0.1
alpha_acc = 0.1

plt.figure("Gyro")
Gyr = DimensionalSplit(g.gyro)
plt.plot(range(len(Gyr[0])), Gyr[0])
plt.plot(range(len(Gyr[1])), Gyr[1])
plt.plot(range(len(Gyr[2])), Gyr[2])

g.gyro = np.asarray(g.gyro)
g.acc_1 = np.asarray(g.acc_1)
g.mag_1 = np.asarray(g.mag_1)
for i in range(len(g.acc_1)):
    g.gyro[i] = alpha_gyro * g.gyro[i] + (1 - alpha_gyro)*Gyro
    Gyro = g.gyro[i]

for i in range(len(g.acc_1)):
    g.acc_1[i] = alpha_acc*  g.acc_1[i] + (1 - alpha_acc)*Accel
    Accel = g.acc_1[i] 

for i in range(len(g.acc_1)):
    g.mag_1[i] = alpha_magn*  g.mag_1[i] + (1 - alpha_magn)*Magnet
    Magnet = g.mag_1[i]

Acc = DimensionalSplit(g.acc_1)
Gyr = DimensionalSplit(g.gyro)
Mag = DimensionalSplit(g.mag_1)


# plt.figure("ACC")
# plt.plot(range(len(Acc[0])), Acc[0])
# plt.plot(range(len(Acc[1])), Acc[1])
# plt.plot(range(len(Acc[2])), Acc[2])


plt.figure()
plt.plot(range(len(Gyr[0])), Gyr[0])

plt.figure()
plt.plot(range(len(Gyr[1])), Gyr[1])

plt.figure()
plt.plot(range(len(Gyr[2])), Gyr[2])

# plt.figure("Mag")
# plt.plot(range(len(Mag[0])), Mag[0])
# plt.plot(range(len(Mag[1])), Mag[1])
# plt.plot(range(len(Mag[2])), Mag[2])

plt.figure("time")
plt.plot(range(len(g.timestamp)),g.timestamp)
plt.show()