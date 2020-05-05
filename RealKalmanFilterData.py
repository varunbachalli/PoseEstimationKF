import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time
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
TimeStamp = []

InitialTriad = np.identity(3)
P_k = np.identity(4)
previousT = 0.0
X_k = np.asarray([1.0,0.0,0.0,0.0])
z_k = np.asarray([1.0,0.0,0.0,0.0])
Q_k = np.identity(4)
Q_k *= 0.1
R_k = np.identity(4)
R_k *= 0.1
K_k =  np.identity(4)

'''
void KalmanFilter::getRPY_1(Eigen::Vector4d q, Eigen::Vector3d& angles)
{
    
    double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    angles = angles * 180.0 / M_PI;
}
'''

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

    angles = angles* 180.0/np.pi
    return angles
        

def RungeKutta4(q_0, T, w):
    W = 0.5 * np.asarray( [ [0.0        ,-w[0]        ,-w[1]      ,-w[2]],
                            [w[0]       ,0.0          ,w[2]       ,-w[1]],   
                            [w[1]       ,-w[2]        ,0.0        , w[0]],
                            [w[2]       ,w[1]         ,-w[0]      , 0.0]])

    # print(W)
    k1 = np.dot(W, q_0)
    k2 = np.dot(W, q_0 + T/2 * k1)
    k3 = np.dot(W, q_0 + T/2 * k2)
    k4 = np.dot(W, q_0 + T* k3)
    # print('k1\t', k1)
    # print('k2\t', k2)
    # print('k3\t', k3)
    # print('k4\t', k4)
    # print(T)
    # print(k1 + 2*k2 + 2*k3 + k4)
    q_k1 = q_0 + 1/6*T*(k1 + 2*k2 + 2*k3 + k4)
    absqk = 0.0
    for i in q_k1:
        absqk += i**2
    absqk = np.sqrt(absqk)
    q_k1 = q_k1/absqk
    # print(q_k1)
    return q_k1



def GetJacobian(w):
    W = 0.5 * np.asarray( [ [1.0        ,-w[0]        ,-w[1]      ,-w[2]],
                            [w[0]       ,1.0          ,w[2]       ,-w[1]],   
                            [w[1]       ,-w[2]        ,1.0        , w[0]],
                            [w[2]       ,w[1]         ,-w[0]      , 1.0]])
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

def norm(a):
    result = 0.0
    for i in a:
        result+= i**2
    return result

def ComputeTriad(s,m):
    t1 = s
    t2 = np.cross(s,m)
    t2 = t2/norm(t2)
    t3 = np.cross(t2,t1)
    R = [[t1[0],t2[0],t3[0]],
         [t1[1],t2[1],t3[1]],
         [t1[2],t2[2],t3[2]]]
    R = np.asarray(R)

    return R



def SetData():
    with open('KalmanFilterData.csv') as file:
        reader = csv.reader(file)
        count = 0
        for row in reader:
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
                TimeStamp.append(float(row[17]))
            count = count +1


def init(startValue):
    global InitialTriad
    global previousT, Mag_x,Mag_y,Mag_z,Acc_x,Acc_y,Acc_z,TimeStamp
    acc_0 = np.asarray([Acc_x[startValue],Acc_y[startValue],Acc_z[startValue]])
    mag_0 = np.asarray([Mag_x[startValue],Mag_y[startValue],Mag_z[startValue]])
    InitialTriad = ComputeTriad(acc_0,mag_0)
    previousT = TimeStamp[startValue]

def Prediction(Gyro, T): # Gyro = 3d np array, 
    global previousT, z_k, K_k,P_k, X_k,R_k
    '''
    Eigen::Matrix<double, 4, 4> Jacobian;
    GetJacobian(Gyro, Jacobian);
    //std::cout << "1\n";
    P_k = Jacobian * P_k * Jacobian.transpose() + Q_k;
    //std::cout << "2\n";
    Eigen::Vector4d PrevX_k = X_k;
    //std::cout << "3\n";
    RungeKuttaEval(X_k, T, Gyro);
    //std::cout << "4\n";
    Eigen::Vector3d Change_in_angle;
    getRPY_1(X_k - PrevX_k , Change_in_angle);
    //std::cout << "5\n";
    /*printf("Change_in_angle is [%f,%f,%f]\n", Change_in_angle(0), Change_in_angle(1), Change_in_angle(2));*/
    /*std::cout << "change in angle is " << Change_in_angle(2) << std::endl;*/
    Eigen::Matrix4d H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    S_k = H * P_k * H.transpose() + R_k;
    //std::cout << "6\n";
    K_k = P_k * H.transpose() * S_k.inverse();
    //std::cout << "7\n";
    z_k = H * X_k;
    previousT = T;
    '''
    
    # print(Gyro)
    
    J = GetJacobian(Gyro)
    P_k = np.matmul(np.matmul(J,P_k),J.transpose())
    X_k = RungeKutta4(X_k,T-previousT,Gyro)
    print(T-previousT)
    S_k = P_k + R_k
    z_k = X_k
    K_k = np.matmul(P_k, S_k.transpose())
    previousT = T
    # print("prediction\t",X_k)

def Correction(Mag,Acc): # Mag = 3d np array, Acc = 3d np array, 
    global InitialTriad,z_k,X_k,K_k,P_k
    '''
    Eigen::Matrix3d Triad_new;
    ComputeTriad(Mag_1, Acc_1, Triad_new);
    //std::cout << "9\n";
    //std::cout << "Newly computed Triad is \n" << Triad_new << std::endl;
    Eigen::Matrix3d RotationMatrix = Triad_new.transpose() * InitialTriad;
    //std::cout << "10\n";
    Eigen::Vector4d Quart;
    RotatationMatrix2Quarternion(RotationMatrix, Quart);
    if (Error_1 > Error_2)
    {
        Error_in_prediction = z_k + Quart;
    }
        
    else
    {
        Error_in_prediction = Quart - z_k;
    }
     X_k = X_k + K_k * (Error_in_prediction);
    X_k = X_k/ X_k.norm();
    Eigen::Matrix4d H;
    H << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    P_k = P_k - K_k * H * P_k;
    //std::cout << "13\n";

    '''
    TriadNew = ComputeTriad(Acc,Mag)
    RotationMatrix = np.matmul(TriadNew.transpose(),InitialTriad)
    Quart = RotationMatrix2Quart(RotationMatrix)
    Error_in_prediction = np.asarray([0.0,0.0,0.0,0.0])
    Error_1 = norm(z_k - Quart)
    Error_2 = norm(z_k + Quart)

    if Error_1 > Error_2:
        Error_in_prediction = z_k + Quart
    else:
        Error_in_prediction = Quart - z_k
    
    X_k = X_k + np.matmul(K_k,Error_in_prediction)
    P_k = P_k - np.matmul(K_k,P_k)
    X_k = X_k/norm(X_k)
    # print("correction \t",X_k)
    # return X_k

def rotate(q, v):
    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]
    R = np.asarray([[a**2 + b**2 - c**2 - d**2 ,2*b*c - 2*a*d               ,2*b*d + 2*a*c],
                    [2*b*c + 2*a*d              ,a**2 - b**2 + c**2 -d**2   ,2*c*d - 2*a*d],
                    [2*b*d- 2*a*c               ,2*c*d + 2*a*b              ,a**2 - b**2 - c**2 +d**2]])

    w_rotate = np.dot(R,v)
    return w_rotate

def get_cube(Quarternion):   # send X_k
    x = np.asarray([[ 1., -1., -1.,  1.,  1.],
                    [ 1., -1., -1.,  1.,  1.],
                    [-1.,  1.,  1., -1., -1.],
                    [-1.,  1.,  1., -1., -1.],
                    [ 1., -1., -1.,  1.,  1.]])
    y = np.asarray([[ 1.,  1., -1., -1.,  1.],
                    [ 1.,  1., -1., -1.,  1.],
                    [-1., -1.,  1.,  1., -1.],
                    [-1., -1.,  1.,  1., -1.],
                    [ 1.,  1., -1., -1.,  1.]])
    z = np.asarray([[ 1.,  1.,  1.,  1.,  1.],
                    [-1., -1., -1., -1., -1.],
                    [-1., -1., -1., -1., -1.],
                    [ 1.,  1.,  1.,  1.,  1.],
                    [ 1.,  1.,  1.,  1.,  1.]])
    
    for i in range(5):
        for j in range(5):
            vec = np.asarray([x[i][j],y[i][j],z[i][j]])
            rot = rotate(Quarternion, vec)
            x[i][j] = rot[0]
            y[i][j] = rot[1]
            z[i][j] = rot[2]
    
    return x,y,z




fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
SetData()
init(0)
def Plotting(i):
    global X_k
    global Acc_x
    global Acc_y
    global Acc_z
    global Mag_x
    global Mag_y
    global Mag_z
    global Gyr_x
    global Gyr_y
    global Gyr_z
    global TimeStamp
    global Quarterion_Correct_w
    global Quarterion_Correct_x
    global Quarterion_Correct_y
    global Quarterion_Correct_z

    a = 2
    b = 3
    c = 1
    Gyro = np.asarray([Gyr_x[i],Gyr_y[i],Gyr_z[i]])
    timestamp = TimeStamp[i]
    Prediction(Gyro,timestamp)

    Magneto = np.asarray([Mag_x[i],Mag_y[i],Mag_z[i]])
    Accelero = np.asarray([Acc_x[i],Acc_y[i],Acc_z[i]])
    Correction(Magneto, Accelero)

    # print(Magneto, ' ', Accelero,' ',X_k , ' ', Quart2RPY(X_k))
    # print(X_k)
    QuartCorrection = np.asarray([Quarterion_Correct_w[i], Quarterion_Correct_x[i],Quarterion_Correct_y[i],Quarterion_Correct_z[i]])
    # print(QuartCorrection)
    x,y,z = get_cube(X_k)
    
    ax.clear()
    ax.plot_surface(x*a, y*b, z*c)
    ax.set_xlim(-5,5)
    ax.set_ylim(-5,5)
    ax.set_zlim(-5,5)

ani = animation.FuncAnimation(fig, Plotting, frames = range(1,1800) ,interval = 0.01)
plt.show()

    






# def Animation(Quarternion):
