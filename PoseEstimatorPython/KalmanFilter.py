import numpy as np 

class KalmanFilter:
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
        for i in q_k1:
            absqk += i**2
        absqk = np.sqrt(absqk)
        q_k1 = q_k1/absqk
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
        if(Ang_x > 180.0 or Ang_x < -180.0):
            Ang_x = -Ang_x
        if(Ang_y > 180.0 or Ang_y < -180.0):
            Ang_y = -Ang_y
        if(Ang_z > 180.0 or Ang_z < -180.0):
            Ang_z = -Ang_z

        Angles_Gyro.append([Ang_x,Ang_y,Ang_z])
        S_k = P_k + R_k
        # print(S_k)
        z_k = X_k
        S_k_inv = np.linalg.inv(S_k) 
        K_k = np.matmul(P_k,np.matmul(P_k, S_k_inv))
        K_k = np.identity(4)*0.01
        K_s.append(np.linalg.det(K_k))
        previousT = T

        

    def Correction(Mag,Acc): # Mag = 3d np array, Acc = 3d np array, 
        global InitialTriad,z_k,X_k,K_k,P_k

        TriadNew = ComputeTriad(Acc,Mag)
        RotationMatrix = np.matmul(TriadNew.transpose(),InitialTriad)
        r = scipyRot.from_matrix(RotationMatrix)
        print(RotationMatrix)
        Quart = r.as_quat()
        # Quart = RotationMatrix2Quart(RotationMatrix)
        Angles_Triad.append(r.as_euler('xyz', degrees=True))
        Error_in_prediction = Quart - z_k
        X_k = X_k + np.matmul(K_k,Error_in_prediction)
        P_k = P_k - np.matmul(K_k,P_k)
        X_k = X_k/norm(X_k)



