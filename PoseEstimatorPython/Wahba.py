import numpy as np 

class Wahba:
    def __init__(self, acc, mag):
        self.w_initial_acc = acc
        self.w_initial_mag = mag

    def getRotation(self,acc, mag, k_acc, k_mag):
        w_rotate_acc = acc
        w_rotate_mag = mag
        B_acc = k_acc * np.outer(w_rotate_acc,self.w_initial_acc)
        B_mag = k_mag * np.outer(w_rotate_mag,self.w_initial_mag)
        B_total = B_acc+B_mag
        u,s,vh = np.linalg.svd(B_total)
        M = np.diag(np.asarray([1,1,np.linalg.det(u)*np.linalg.det(vh)]))
        rotate = np.matmul(np.matmul(u,M),vh)
        return rotate

    @staticmethod
    def RotationMatrix2Quart(M):
        tr1 = 1.0 + M[0][0] - M[1][1] - M[2][2]
        tr2 = 1.0 - M[0][0] + M[1][1] - M[2][2]
        tr3 = 1.0 - M[0][0] - M[1][1] + M[2][2]
        qw = 0.0
        qx = 0.0
        qy = 0.0
        qz = 0.0
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

    def getQuarternion(self,acc, mag, k_acc, k_mag):
        return Wahba.RotationMatrix2Quart(self.getRotation(acc, mag, k_acc, k_mag))
