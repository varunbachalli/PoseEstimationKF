from Wahba import Wahba
import numpy as np 
from UtilityFunctions import norm

class KalmanFilter:
    def __init__(self, T0, mag_0, acc_0,eps):
        self.previousT = T0
        self.wahba = Wahba(acc_0,mag_0)
        self.Q = np.identity(3)
        self.R = np.identity(4)
        self.eps = eps
    def setQ(self,q):
        self.Q *= q
    def setR(self,r):
        self.R *= r

    @staticmethod
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
    
    def GetJacobian_A(self,w):
        W = 0.5 * np.asarray( [ [0.0        ,-w[0]        ,-w[1]      ,-w[2]],
                                [w[0]       ,0.0          ,w[2]       ,-w[1]],   
                                [w[1]       ,-w[2]        ,0.0        , w[0]],
                                [w[2]       ,w[1]         ,-w[0]      , 0.0]])
        return W
    
    
    def GetJacobian_B(self,q):
        W = 0.5 * np.asarray( [ [-q[1],     -q[2],    -q[3]],
                                [ q[0],      q[3],    -q[2]],   
                                [-q[3],      q[0],     q[1]],
                                [ q[2],     -q[1],     q[0]]])
        return W

    def Prediction(self,Gyro, T, X_k, P_k):
        J_a = self.GetJacobian_A(Gyro)
        J_b = self.GetJacobian_B(X_k)
        P_k = np.matmul(np.matmul(J_a,P_k),J_a.transpose()) + np.matmul(np.matmul(J_b, self.Q),J_b.transpose())
        X_k = KalmanFilter.RungeKutta4(X_k,T-self.previousT,Gyro)
        S_k = P_k + self.R
        z_k = X_k
        S_k_inv = np.linalg.inv(S_k) 
        K_k = np.matmul(P_k,S_k_inv)
        self.previousT = T
        return z_k, P_k, K_k

    def Correction(self,Mag, Acc, z_k, P_k, K_k): 
        Quart = self.wahba.getQuarternion(Acc,Mag,0.5,0.5)
        # Error_in_prediction = []
        comparator  = np.dot(Quart,z_k)
        if(comparator<=1.0 and comparator >= 0.5):
            Error_in_prediction = Quart - z_k
        if(comparator>=-1.0 and comparator <= -0.5):
            Error_in_prediction = -Quart - z_k
        else:
            Error_in_prediction = np.zeros(4)

        X_k = z_k + np.matmul(K_k,Error_in_prediction)
        P_k = P_k - np.matmul(K_k,P_k)
        X_k = X_k/norm(X_k)
        return X_k, P_k
    