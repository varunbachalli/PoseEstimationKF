import numpy as np

from scipy.spatial.transform import Rotation as R

from pyquaternion import Quaternion
# r= R.from_matrix([[ 9.99201637e-01, -3.99510890e-02, -2.85952295e-19],
#  [ 3.99510890e-02,  9.99201637e-01,  1.56638191e-18],
#  [ 4.85065044e-19, -2.47070100e-18,  1.00000000e+00]])

# print(r.as_euler('xyz'))

def norm(a):
    result = 0.0
    for i in a:
        result+= i**2
    result = np.sqrt(result)
    return result

def RotationMatrix2Quart(M):
    tr = M[0][0] + M[1][1] + M[2][2]
    if (tr > 0): 
        S = np.sqrt(tr+1.0) * 2 # S=4*qw 
        qw = 0.25 * S
        qx = (M[2][1] - M[1][2]) / S
        qy = (M[0][2] - M[2][0]) / S
        qz = (M[1][0] - M[0][1]) / S 
        print(1)
    
    elif M[0][0] > M[1][1] and M[0][0] > M[2][2]: 
        S = np.sqrt(1.0 + M[0][0] - M[1][1] - M[2][2]) * 2 # S=4*qx 
        qw = (M[2][1] - M[1][2]) / S
        qx = 0.25 * S
        qy = (M[0][1] + M[1][0]) / S 
        qz = (M[0][2] + M[2][0]) / S
        print(2)
    elif M[1][1] > M[2][2]:
        S = np.sqrt(1.0 + M[1][1] - M[0][0] - M[2][2]) * 2 # S=4*qy
        qw = (M[0][2] - M[2][0]) / S
        qx = (M[0][1] + M[1][0]) / S 
        qy = 0.25 * S
        qz = (M[1][2] + M[2][1]) / S
        print(3)
    else:
        S = np.sqrt(1.0 + M[2][2] - M[0][0] - M[1][1]) * 2 # S=4*qz
        qw = (M[1][0] - M[0][1]) / S
        qx = (M[0][2] + M[2][0]) / S
        qy = (M[1][2] + M[2][1]) / S
        qz = 0.25 * S
        print(4)
    return np.asarray([qw,qx,qy,qz])

# def QuarternionRotation_1(q,v):
#     Q = np.asarray([[-q[1], q[0] ,-q[3],  q[2]],
#                     [-q[2], q[3],  q[0], -q[1]],
#                     [-q[3],-q[2],  q[1],  q[0]]])
#     return np.matmul(Q,v)

# def def_rotate(q, v):
#     a = q[0]
#     b = q[1]
#     c = q[2]
#     d = q[3]
#     s = norm(q)
#     R = np.asarray([[a**2 + b**2 - c**2 - d**2  ,2*b*c - 2*a*d               ,2*b*d + 2*a*c],
#                     [2*b*c + 2*a*d              ,a**2 - b**2 + c**2 -d**2    ,2*c*d - 2*a*b],
#                     [2*b*d- 2*a*c               ,2*c*d + 2*a*b               ,a**2 - b**2 - c**2 +d**2]])
#     print(R)
#     w_rotate = np.matmul(R,v)
#     return w_rotate

def get_rotation_matrix(q):
    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]
    s = norm(q)
    R = np.asarray([[a**2 + b**2 - c**2 - d**2  ,2*b*c - 2*a*d               ,2*b*d + 2*a*c],
                    [2*b*c + 2*a*d              ,a**2 - b**2 + c**2 -d**2    ,2*c*d - 2*a*b],
                    [2*b*d- 2*a*c               ,2*c*d + 2*a*b               ,a**2 - b**2 - c**2 +d**2]])
    
    # print(R)
    # w_rotate = np.matmul(R,v)
    return R

# def quaternion_multiply(quaternion1, quaternion0):
#     w0, x0, y0, z0 = quaternion0
#     w1, x1, y1, z1 = quaternion1
#     return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
#                      x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
#                      -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
#                      x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

# def DCM2Quart(M):
#     v1 = np.asarray([1.0,0.0,0.0])
#     axis_1 = np.cross(v1, M[:,0])
#     axis_1 = axis_1/norm(axis_1)
#     angle_1 = np.arccos(M[0,0])
#     axis_1 = np.sin(angle_1/2)*axis_1
#     Quart1 = np.asarray([np.cos(angle_1/2),axis_1[0],axis_1[1],axis_1[2]])
#     v2R = rotate(Quart1,np.asarray([0.,1.,0.]))
#     v2R = v2R/norm(v2R)
#     angle_2 = np.arccos(np.dot(v2R, M[:,1]))
#     axis_2 = np.asarray([np.sin(angle_2/2),0.0,0.0])
#     Quart2 = np.asarray([np.cos(angle_2/2),axis_2[0],axis_2[1],axis_2[2]])
#     Quartf = quaternion_multiply(Quart1,Quart2)
#     return Quartf



# g = np.asarray([0.0, 1.0, 1.0, 1.0])

# theta = 90*np.pi/180.0
# q = ([np.cos(theta), np.sin(theta),0.0,0.0])

# g_r = QuarternionRotation(q,g[1:4])

# print(g_r)

# theta = 90 * np.pi/180
# g = np.asarray([1.0,1.0,1.0])
# q = Quaternion(axis = np.asarray([1.0,0.0,0.0]), angle = theta)
# g_ = q.rotate(g)
# print(g_)

# r = R.from_quat([np.sin(theta/2), 0, 0,  np.cos(theta/2)])
# g_ = r.apply(g)
# print(g_)

# r_rot = r.as_matrix()
# print(r_rot)

# q_ = np.asarray([np.cos(theta/2),np.sin(theta/2), 0, 0])
# g_ = def_rotate(q,g)
# print(g_)


for i in range(100000):
    q = np.random.rand(4)
    q = q/norm(q)
    q2_rot = get_rotation_matrix(q)
    r2_quat = RotationMatrix2Quart(q2_rot)

    # r = R.from_quat(q)
    # r_mat = r.as_matrix()
    # r2 = R.from_matrix(r_mat)
    # r2_quat = r2.as_quat()
    eps = np.asarray([0.00000000000001]*4)
    if(not ((r2_quat - q<=eps).any() or (r2_quat-q >=-eps).any())):
        print("failure")
        print(r2_quat)
        print(q)
        
 


# create random quarternion , rotate a random vector, convert quarternion to rotation matrix, back to quarternion, rotate same vector, check if they match.







