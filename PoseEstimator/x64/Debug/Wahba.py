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


