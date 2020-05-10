import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as scipyRot
from Wahba import Wahba

Acc_x = []
Acc_y = []
Acc_z = []
Mag_x = []
Mag_y = []
Mag_z = []
Gyr_x = []
Gyr_y = []
Gyr_z = []
Quarterion_Predict_w = []	
Quarterion_Predict_x = []	
Quarterion_Predict_y = []	
Quarterion_Predict_z = []	
Quarterion_Correct_w = []	
Quarterion_Correct_x = []	
Quarterion_Correct_y = []	
Quarterion_Correct_z = []
Quarterion_Gyro_w = []	
Quarterion_Gyro_x = []	
Quarterion_Gyro_y = []	
Quarterion_Gyro_z = []

Quarterion_Wahba_w = []	
Quarterion_Wahba_x = []	
Quarterion_Wahba_y = []	
Quarterion_Wahba_z = []

Quarterion_Filter_w = []	
Quarterion_Filter_x = []	
Quarterion_Filter_y = []	
Quarterion_Filter_z = []



TimeStamp = []

Angles_Gyro = [[0.0,0.0,0.0]]
Angles_Filter = [[0.0,0.0,0.0]]
Angles_Triad = [[0.0,0.0,0.0]]
Angles_Filter_original = [[0.0,0.0,0.0]]
Angles_Filter_gyro = [[0.0,0.0,0.0]]

InitialTriad = np.identity(3)
P_k = np.identity(4)
previousT = 0.0
X_k = np.asarray([1.0,0.0,0.0,0.0])
z_k = np.asarray([1.0,0.0,0.0,0.0])
Q_k = np.identity(3)
Q_k *= 0.01
R_k = np.identity(4)
R_k *= 0.1
K_k =  np.identity(4)

K_s = []

wahba = Wahba(np.asarray(Angles_Gyro),np.asarray(Angles_Gyro))

def rotate(q, v):
    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]
    R = np.asarray([[a**2 + b**2 - c**2 - d**2 ,2*b*c - 2*a*d               ,2*b*d + 2*a*c],
                    [2*b*c + 2*a*d              ,a**2 - b**2 + c**2 -d**2   ,2*c*d - 2*a*b],
                    [2*b*d- 2*a*c               ,2*c*d + 2*a*b              ,a**2 - b**2 - c**2 +d**2]])
    w_rotate = np.dot(R,v)
    return w_rotate


def Quart2RPY(q):
    angles = np.asarray([0.0,0.0,0.0])
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    angles[0] = np.arctan2(sinr_cosp, cosr_cosp)
    angles[1] = np.arcsin(sinp)
    siny_cosp = 2 * (q[0] * q[3] + q[1]* q[2])
    cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    angles[2] = np.arctan2(siny_cosp, cosy_cosp)
    angles = angles * 180.0/np.pi
    return angles

def RungeKutta4(q_0, T, w):
    W = 0.5 * np.asarray( [ [0.0        ,-w[0]        ,-w[1]      ,-w[2]],
                            [w[0]       ,0.0          ,w[2]       ,-w[1]],   
                            [w[1]       ,-w[2]        ,0.0        , w[0]],
                            [w[2]       ,w[1]         ,-w[0]      , 0.0]])

    T = T*(10**-9)
    k1 = np.dot(W, q_0)
    k2 = np.dot(W, q_0 + T/2 * k1)
    k3 = np.dot(W, q_0 + T/2 * k2)
    k4 = np.dot(W, q_0 + T* k3)

    q_k1 = q_0 + 1/6*T*(k1 + 2*k2 + 2*k3 + k4)

    absqk = 0.0
    q_k1 = q_k1/norm(q_k1)
    return q_k1



def GetJacobian_A(w):
    W = 0.5 * np.asarray( [ [0.0        ,-w[0]        ,-w[1]      ,-w[2]],
                            [w[0]       ,0.0          ,w[2]       ,-w[1]],   
                            [w[1]       ,-w[2]        ,0.0        , w[0]],
                            [w[2]       ,w[1]         ,-w[0]      , 0.0]])
    return W

def GetJacobian_B(q):
    W = 0.5 * np.asarray( [ [-q[1],     -q[2],    -q[3]],
                            [ q[0],      q[3],    -q[2]],   
                            [-q[3],      q[0],     q[1]],
                            [ q[2],     -q[1],     q[0]]])
    return W


def RotationMatrix2Quart(M):
    tr = M[0][0] + M[1][1] + M[2][2]
    if (tr > 0): 
        S = np.sqrt(tr+1.0) * 2 # S=4*qw 
        qw = 0.25 * S
        qx = (M[2][1] - M[1][2]) / S
        qy = (M[0][2] - M[2][0]) / S
        qz = (M[1][0] - M[0][1]) / S 

    elif M[0][0] > M[1][1] and M[0][0] > M[2][2]: 
        S = np.sqrt(1.0 + M[0][0] - M[1][1] - M[2][2]) * 2 # S=4*qx 
        qw = (M[2][1] - M[1][2]) / S
        qx = 0.25 * S
        qy = (M[0][1] + M[1][0]) / S 
        qz = (M[0][2] + M[2][0]) / S
    elif M[1][1] > M[2][2]:
        S = np.sqrt(1.0 + M[1][1] - M[0][0] - M[2][2]) * 2 # S=4*qy
        qw = (M[0][2] - M[2][0]) / S
        qx = (M[0][1] + M[1][0]) / S 
        qy = 0.25 * S
        qz = (M[1][2] + M[2][1]) / S
    else:
        S = np.sqrt(1.0 + M[2][2] - M[0][0] - M[1][1]) * 2 # S=4*qz
        qw = (M[1][0] - M[0][1]) / S
        qx = (M[0][2] + M[2][0]) / S
        qy = (M[1][2] + M[2][1]) / S
        qz = 0.25 * S
    return np.asarray([qw,qx,qy,qz])

def RotationMatrix2Quart_1(M):
    tr1 = 1.0 + M[0][0] - M[1][1] - M[2][2]
    tr2 = 1.0 - M[0][0] + M[1][1] - M[2][2]
    tr3 = 1.0 - M[0][0] - M[1][1] + M[2][2]

    if (tr1 > tr2)and(tr1 > tr3):
        S = np.sqrt(tr1) * 2; # S=4*qx 
        qw = (M[2][1] - M[1][2]) / S
        qx = 0.25 * S
        qy = (M[0][1] + M[1][0]) / S
        qz = (M[0][2] + M[2][0]) / S 

    elif (tr2 > tr1) and (tr2 > tr3):
        S = np.sqrt(tr2) * 2 # S=4*qy
        qw = (M[0][2] - M[2][0]) / S
        qx = (M[0][1] + M[1][0]) / S 
        qy = 0.25 * S
        qz = (M[1][2] + M[2][1]) / S 
    else:
        S = np.sqrt(tr3) * 2; # S=4*qz
        qw = (M[1][0] - M[0][1]) / S
        qx = (M[0][2] + M[2][0]) / S
        qy = (M[1][2] + M[2][1]) / S
        qz = 0.25 * S
    return np.asarray([qw,qx,qy,qz])
    

def norm(a):
    result = 0.0
    for i in a:
        result+= i**2
    result = np.sqrt(result)
    return result

# def ComputeTriad(s,m):
#     t1 = s
#     t2 = np.cross(s,m)
#     t2 = t2/norm(t2)
#     t3 = np.cross(t2,t1)
#     R = [[t3[0],t2[0],t1[0]],
#          [t3[1],t2[1],t1[1]],
#          [t3[2],t2[2],t1[2]]]
#     R = np.asarray(R)
#     return R


def SetData():
    with open('KalmanFilterData.txt') as file:
        reader = csv.reader(file)
        count = 0
        for row in reader:
            print(row)
            if(count > 0):
                Acc_x.append(float(row[0]))
                Acc_y.append(float(row[1]))
                Acc_z.append(float(row[2]))
                Mag_x.append(float(row[3]))
                Mag_y.append(float(row[4]))
                Mag_z.append(float(row[5]))
                Gyr_x.append(float(row[6]))
                Gyr_y.append(float(row[7]))
                Gyr_z.append(float(row[8]))
                Quarterion_Predict_w.append(float(row[9]))
                Quarterion_Predict_x.append(float(row[10]))
                Quarterion_Predict_y.append(float(row[11]))
                Quarterion_Predict_z.append(float(row[12]))
                Quarterion_Correct_w.append(float(row[13]))
                Quarterion_Correct_x.append(float(row[14]))
                Quarterion_Correct_y.append(float(row[15]))
                Quarterion_Correct_z.append(float(row[16]))
                Quarterion_Gyro_w.append(float(row[17]))
                Quarterion_Gyro_x.append(float(row[18]))
                Quarterion_Gyro_y.append(float(row[19]))
                Quarterion_Gyro_z.append(float(row[20]))
                TimeStamp.append(float(row[21]))
            count = count +1
    print(len(Mag_x))
    print(len(Acc_z))

def init(startValue):
    global wahba
    global previousT, Mag_x,Mag_y,Mag_z,Acc_x,Acc_y,Acc_z,TimeStamp
    acc_0 = np.asarray([Acc_x[startValue],Acc_y[startValue],Acc_z[startValue]])
    acc_0 = acc_0/norm(acc_0)
    mag_0 = np.asarray([Mag_x[startValue],Mag_y[startValue],Mag_z[startValue]])
    mag_0 = mag_0/norm(mag_0)
    # InitialTriad = ComputeTriad(acc_0,mag_0)
    wahba = Wahba(acc_0,mag_0)
    previousT = TimeStamp[startValue]

def Prediction(Gyro, T): # Gyro = 3d np array, 
    global previousT, z_k, K_k,P_k, X_k,R_k
    global K_s

    J_a = GetJacobian_A(Gyro)
    J_b = GetJacobian_B(X_k)
    P_k = np.matmul(np.matmul(J_a,P_k),J_a.transpose()) + np.matmul(np.matmul(J_b, Q_k),J_b.transpose())
    X_k = RungeKutta4(X_k,T-previousT,Gyro)
    Ang_x = Angles_Gyro[-1][0] + Gyro[0]*(T-previousT)/10**9*180/np.pi 
    Ang_y = Angles_Gyro[-1][1] + Gyro[1]*(T-previousT)/10**9*180/np.pi
    Ang_z = Angles_Gyro[-1][2] + Gyro[2]*(T-previousT)/10**9*180/np.pi
    if(Ang_x > 183.0 or Ang_x < -183.0):
        Ang_x = -Ang_x
    if(Ang_y > 183.0 or Ang_y < -183.0):
        Ang_y = -Ang_y
    if(Ang_z > 183.0 or Ang_z < -183.0):
        Ang_z = -Ang_z

    Angles_Gyro.append([Ang_x,Ang_y,Ang_z])
    S_k = P_k + R_k
    # print(S_k)
    z_k = X_k
    S_k_inv = np.linalg.inv(S_k) 
    K_k = np.matmul(P_k,S_k_inv)
    # K_k = np.identity(4)*0.01
    K_s.append(np.linalg.det(K_k))
    previousT = T

    

def Correction(Mag,Acc): # Mag = 3d np array, Acc = 3d np array, 
    global wahba,z_k,X_k,K_k,P_k
    RotationMatrix = wahba.getRotation(Acc,Mag,0.5,0.5)#abs(Acc[2]), 1- abs(Acc[2]))
    # r = scipyRot.from_matrix(RotationMatrix.transpose())
    Quart = RotationMatrix2Quart_1(RotationMatrix.transpose())
    
    Quarterion_Wahba_w.append(Quart[0])
    Quarterion_Wahba_x.append(Quart[1])
    Quarterion_Wahba_y.append(Quart[2])
    Quarterion_Wahba_z.append(Quart[3])

    # Quart = RotationMatrix2Quart(RotationMatrix)
    # Angles_Triad.append(r.as_euler('xyz', degrees=True))
    Error_in_prediction = Quart - z_k
    X_k = X_k + np.matmul(K_k,Error_in_prediction)
    P_k = P_k - np.matmul(K_k,P_k)
    X_k = X_k/norm(X_k)

    Quarterion_Filter_w.append(X_k[0])
    Quarterion_Filter_x.append(X_k[1])
    Quarterion_Filter_y.append(X_k[2])
    Quarterion_Filter_z.append(X_k[3])




def get_cube(Quarternion):   # send X_k
    x = np.sqrt(1/3)*np.asarray([[ 1., -1., -1.,  1.,  1.],
                                 [ 1., -1., -1.,  1.,  1.],
                                 [-1.,  1.,  1., -1., -1.],
                                 [-1.,  1.,  1., -1., -1.],
                                 [ 1., -1., -1.,  1.,  1.]])
    y = np.sqrt(1/3)*np.asarray([[ 1.,  1., -1., -1.,  1.],
                                 [ 1.,  1., -1., -1.,  1.],
                                 [-1., -1.,  1.,  1., -1.],
                                 [-1., -1.,  1.,  1., -1.],
                                 [ 1.,  1., -1., -1.,  1.]])
    z = np.sqrt(1/3)*np.asarray([[ 1.,  1.,  1.,  1.,  1.],
                                 [-1., -1., -1., -1., -1.],
                                 [-1., -1., -1., -1., -1.],
                                 [ 1.,  1.,  1.,  1.,  1.],
                                 [ 1.,  1.,  1.,  1.,  1.]])
    
    for i in range(5):
        for j in range(5):
            vec = np.asarray([x[i][j],y[i][j],z[i][j]])
            Quart = Quaternion(Quarternion[0],Quarternion[1],Quarternion[2],Quarternion[3])
            rot = Quart.rotate(vec)
            x[i][j] = rot[0]#/np.sqrt(3)
            y[i][j] = rot[1]#/np.sqrt(3)
            z[i][j] = rot[2]#/np.sqrt(3)
    
    return x,y,z

# def getSquare(x,y,z,Quart):
#     for i in range(len(x)):
#         v = np.asarray([0.0,x[i],y[i],z[i]])
#         rot = rotate_(Quart,v)
#         x[i] = rot[0]
#         y[i] = rot[1]
#         z[i] = rot[2]
#     return x,y,z

# fig = plt.figure()
# ax = fig.add_subplot(211, projection='3d')
# ax2 = fig.add_subplot(212, projection = '3d')
# ax_xy = fig.add_subplot(221)
# ax_yz = fig.add_subplot(222)
# ax_zx = fig.add_subplot(223)



SetData()
init(0)



for i in range(len(Gyr_x)):
    Gyro = np.asarray([Gyr_x[i],Gyr_y[i],Gyr_z[i]])
    timestamp = TimeStamp[i]
    Prediction(Gyro,timestamp)
    Magneto = np.asarray([Mag_x[i-1],Mag_y[i-1],Mag_z[i-1]])
    Magneto = Magneto/norm(Magneto)
    Accelero = np.asarray([Acc_x[i],Acc_y[i],Acc_z[i]])
    Accelero = Accelero/norm(Accelero)
    Correction(Magneto, Accelero) 
    QuartCorrection = np.asarray([Quarterion_Predict_w[i], Quarterion_Predict_x[i],Quarterion_Predict_y[i],Quarterion_Predict_z[i]])
    r = scipyRot.from_quat(X_k)
    
    Angles_Filter.append(r.as_euler('xyz', degrees=True))

    Angles_Filter_original.append(Quart2RPY(QuartCorrection).tolist())

    Quart_Gyro = np.asarray([Quarterion_Gyro_w[i], Quarterion_Gyro_x[i],Quarterion_Gyro_y[i],Quarterion_Gyro_z[i]])
    Angles_Filter_gyro.append(Quart2RPY(Quart_Gyro).tolist())

# print(len(Angles_Filter))
# print(len(Angles_Gyro))
# print(len(Angles_Triad))


Angles_Filter_roll = []
# Angles_Filter_original_roll = []
# Angles_Filter_gyro_roll = []
# Angles_Gyro_roll = []
# Angles_Triad_roll = []
Angles_Filter_pitch = []
# Angles_Filter_original_pitch= []
# Angles_Filter_gyro_pitch = []
# Angles_Gyro_pitch = []
# Angles_Triad_pitch = []
Angles_Filter_yaw = []
# Angles_Filter_original_yaw = []
# Angles_Filter_gyro_yaw = []
# Angles_Gyro_yaw = []
# Angles_Triad_yaw = []

for i in range(len(Angles_Filter)):
    Angles_Filter_roll.append(Angles_Filter[i][0])
    Angles_Filter_pitch.append(Angles_Filter[i][1])
    Angles_Filter_yaw.append(Angles_Filter[i][2])

#     Angles_Filter_original_roll.append(Angles_Filter_original[i][0])
#     Angles_Filter_original_pitch.append(Angles_Filter_original[i][1])
#     Angles_Filter_original_yaw.append(Angles_Filter_original[i][2])

#     Angles_Filter_gyro_roll.append(Angles_Filter_gyro[i][0])
#     Angles_Filter_gyro_pitch.append(Angles_Filter_gyro[i][1])
#     Angles_Filter_gyro_yaw.append(Angles_Filter_gyro[i][2])

#     Angles_Gyro_roll.append(Angles_Gyro[i][0])
#     Angles_Gyro_pitch.append(Angles_Gyro[i][1])
#     Angles_Gyro_yaw.append(Angles_Gyro[i][2])

#     Angles_Triad_roll.append(Angles_Triad[i][0])
#     Angles_Triad_pitch.append(Angles_Triad[i][1])
#     Angles_Triad_yaw.append(Angles_Triad[i][2])

# plt.figure('roll')
# plt.plot(range(len(Angles_Filter)),Angles_Filter_roll,color = 'red', label = 'Angles_Filter_roll')
# # plt.plot(range(len(Angles_Filter_original)),Angles_Filter_original_roll,color = 'black', label = 'Angles_Filter_original')
# # plt.plot(range(len(Angles_Filter_gyro_roll)),Angles_Filter_gyro_roll,color = 'blue', label = 'Angles_Filter_gyro_original')
# plt.plot(range(len(Angles_Filter)),Angles_Gyro_roll,color = 'blue', label = 'Angles_Gyro_roll')
# plt.plot(range(len(Angles_Filter)),Angles_Triad_roll,color = 'green', label = 'Angles_Triad_roll')
# plt.legend()
# plt.figure('pitch')
# plt.plot(range(len(Angles_Filter)),Angles_Filter_pitch,color = 'red',  label = 'Angles_Filter_pitch')
# # plt.plot(range(len(Angles_Filter_original)),Angles_Filter_original_pitch,color = 'black', label = 'Angles_Filter_original_pitch')
# # plt.plot(range(len(Angles_Filter_gyro_pitch)),Angles_Filter_gyro_pitch,color = 'blue', label = 'Angles_Filter_gyro_original')
# plt.plot(range(len(Angles_Filter)),Angles_Gyro_pitch,color = 'blue',  label = 'Angles_Gyro_pitch')
# plt.plot(range(len(Angles_Filter)),Angles_Triad_pitch,color = 'green',  label = 'Angles_Triad_pitch')
# plt.legend()
# plt.figure('yaw')
# plt.plot(range(len(Angles_Filter)),Angles_Filter_yaw,color = 'red', label = 'Angles_Filter_yaw')
# # plt.plot(range(len(Angles_Filter_original)),Angles_Filter_original_yaw,color = 'black', label = 'Angles_Filter_original_yaw')
# # plt.plot(range(len(Angles_Filter_gyro_yaw)),Angles_Filter_gyro_yaw,color = 'blue', label = 'Angles_Filter_gyro_original')
# plt.plot(range(len(Angles_Filter)),Angles_Gyro_yaw,color = 'blue', label = 'Angles_Gyro_yaw')
# plt.plot(range(len(Angles_Filter)),Angles_Triad_yaw,color = 'green', label = 'Angles_Triad_yaw')
# plt.legend()


plt.figure('W')
# plt.plot(range(len(Quarterion_Wahba_w)),Quarterion_Wahba_w,color = 'red', label = 'Wahba')
plt.plot(range(len(Quarterion_Filter_w)),Quarterion_Filter_w,color = 'blue', label = 'Filter')
plt.plot(range(len(Quarterion_Gyro_w)),Quarterion_Gyro_w,color = 'green', label = 'Gyro')
plt.plot(range(len(Quarterion_Correct_w)),Quarterion_Correct_w, color = 'black', label = 'real')
plt.legend()

plt.figure('X')
# plt.plot(range(len(Quarterion_Wahba_x)),Quarterion_Wahba_x,color = 'red', label = 'Wahba')
plt.plot(range(len(Quarterion_Filter_x)),Quarterion_Filter_x,color = 'blue', label = 'Filter')
plt.plot(range(len(Quarterion_Gyro_x)),Quarterion_Gyro_x,color = 'green', label = 'Gyro')
plt.plot(range(len(Quarterion_Correct_x)),Quarterion_Correct_x, color = 'black', label = 'real')
plt.legend()

plt.figure('Y')
# plt.plot(range(len(Quarterion_Wahba_y)),Quarterion_Wahba_y,color = 'red', label = 'Wahba')
plt.plot(range(len(Quarterion_Filter_y)),Quarterion_Filter_y,color = 'blue', label = 'Filter')
plt.plot(range(len(Quarterion_Gyro_y)),Quarterion_Gyro_y,color = 'green', label = 'Gyro')
plt.plot(range(len(Quarterion_Correct_y)),Quarterion_Correct_y, color = 'black', label = 'real')
plt.legend()

plt.figure('Z')
# plt.plot(range(len(Quarterion_Wahba_z)),Quarterion_Wahba_z,color = 'red', label = 'Wahba')
plt.plot(range(len(Quarterion_Filter_z)),Quarterion_Filter_z,color = 'blue', label = 'Filter')
plt.plot(range(len(Quarterion_Gyro_z)),Quarterion_Gyro_z,color = 'green', label = 'Gyro')
plt.plot(range(len(Quarterion_Correct_z)),Quarterion_Correct_z, color = 'black', label = 'real')
plt.legend()

# plt.figure("Roll Pitch Yaw")

plt.figure()
plt.plot(range(len(K_s)), K_s)
plt.show()
# TODO: 1. In a simulation , plot how the triads rotate [1,0,0], how the filter rotates it and how the Angular velocity rotates it.




# def Plotting(i):
#     global X_k
#     global Acc_x
#     global Acc_y
#     global Acc_z
#     global Mag_x
#     global Mag_y
#     global Mag_z
#     global Gyr_x
#     global Gyr_y
#     global Gyr_z
#     global TimeStamp
#     global Quarterion_Correct_w
#     global Quarterion_Correct_x
#     global Quarterion_Correct_y
#     global Quarterion_Correct_z

#     a = 1
#     b = 1
#     c = 1
#     Gyro = np.asarray([Gyr_x[i],Gyr_y[i],Gyr_z[i]])
#     timestamp = TimeStamp[i]
#     Prediction(Gyro,timestamp)
    
#     Magneto = np.asarray([Mag_x[i-1],Mag_y[i-1],Mag_z[i-1]])
#     Magneto = Magneto/norm(Magneto)
#     Accelero = np.asarray([Acc_x[i],Acc_y[i],Acc_z[i]])
#     Accelero = Accelero/norm(Accelero)
#     Correction(Magneto, Accelero)

#     QuartCorrection = np.asarray([Quarterion_Correct_w[i], Quarterion_Correct_x[i],Quarterion_Correct_y[i],Quarterion_Correct_z[i]])

#     Quart = QuartCorrection
#     x,y,z = get_cube(Quart)
#     ax.clear()
#     ax.plot_surface(x*a, y*b, z*c, color = "blue")
#     ax.set_xlim(-2,2)
#     ax.set_ylim(-2,2)
#     ax.set_zlim(-2,2)

#     Quart = X_k
#     x,y,z = get_cube(Quart)
#     # ax.plot_surface(x*a, y*b, z*c, color = "red")
#     ax2.scatter(x*a, y*b, z*c, color = "red")
#     ax2.set_xlim(-2,2)
#     ax2.set_ylim(-2,2)
#     ax2.set_zlim(-2,2)
#     print(i)
    

# ani = animation.FuncAnimation(fig, Plotting, frames = range(1,842) ,interval = 0.01)
# plt.show()









# def Animation(Quarternion):
