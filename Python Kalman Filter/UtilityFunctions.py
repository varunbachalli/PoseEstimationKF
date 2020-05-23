import numpy as np 

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

def norm(a):
    result = 0.0
    for i in a:
        result+= i**2
    result = np.sqrt(result)
    return result


def DimensionalSplit(S):
    size_1 = len(S)
    size_2 = len(S[0])
    A = []
    count = 0
    for i in range(size_2):
        B = []
        for j in range(size_1):
            B.append(S[j][i])
        A.append(B)     
    return A

