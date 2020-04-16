import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def rotate(q, v):
    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]
    R = np.asarray([[a**2 + b**2 - c**2 - d**2 ,2*b*c - 2*a*d               ,2*b*d + 2*a*c],
                    [2*b*c + 2*a*d              ,a**2 - b**2 + c**2 -d**2   ,2*c*d - 2*a*d],
                    [2*b*d- 2*a*c               ,2*c*d + 2*a*b              ,a**2 - b**2 - c**2 +d**2]])

    # w = np.asarray([0.0,v[0],v[1],v[2]])
    w_rotate = np.dot(R,v)
    #return np.asarray(w_rotate[1],w_rotate[2],w_rotate[3])
    return w_rotate

bo = True

def equation(q_0, T, w):
    W = 0.5 * np.asarray( [ [0.0        ,-w[0]        ,-w[1]      ,-w[2]],
                            [w[0]       ,0.0          ,w[2]       ,-w[1]],   
                            [w[1]       ,-w[2]        ,0.0        , w[0]],
                            [w[2]       ,w[1]         ,-w[0]      , 0.0]])

    # print(W)
    # W_2 = np.dot(W,W)
    # W_3 = np.dot(W_2,W)
    # W_4 = np.dot(W_3,W)
    # q_k1 = q_0 + T*np.dot(W,q_0) + (T**2)/2.0 * np.dot(W_2,q_0) #+ (T**3)/6.0 * np.dot(W_3,q_0) + (T**4)/24.0 * np.dot(W_4,q_0)
    k1 = np.dot(W,q_0)
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
    

    


q0 = np.asarray([1.0,0.0,0.0,0.0])

angular_velocity = np.asarray([3*np.pi/2,np.pi,np.pi/2])

point = np.asarray([1.0,0.0,0.0]) # change the point to [0,1,0] # 0.0 -1.0 -1.0

error = []

# for j in range(5):
#     for i in range(10**j):
#         T = 1.0/(10.0**j)
        
#         q0 = equation(q0,T,angular_velocity)
#         point = rotate(q0,point)
#     error.append(np.linalg.norm(point - np.asarray([0.0,1.0,0.0]),2))



# plt.plot(range(5),error)
# plt.show()
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

points_x = []
points_y = []
points_z = []

numberIters = 1000
T = 1.0/numberIters


points_x.append(point[0])
points_y.append(point[1])
points_z.append(point[2])
print(point)
change = []
for j in range(7):
    numberIters = 10**j
    q0 = np.asarray([1.0,0.0,0.0,0.0])
    point = np.asarray([1.0,0.0,0.0]) 
    print('j', j, end = '')
    T = 1.0/numberIters
    for i in range(numberIters):
        # q1 = equation(q0,T,angular_velocity)
        # change.append(np.linalg.norm(q1- q0,2))
        # q0 = q1
        q0 = equation(q0,T,angular_velocity)
    print(' q0', q0, end = '')
    point = rotate(q0,point)
    print(' point', point)
    # point = rotate(q0,point)
    # points_x.append(point[0])
    # points_y.append(point[1])
    # points_z.append(point[2])
    

# plt.plot(range(numberIters),change)
# plt.show()


# plt.plot(points_x,points_y)
# plt.axis('equal')
# plt.show()
# u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
# x = np.cos(u)*np.sin(v)
# y = np.sin(u)*np.sin(v)
# z = np.cos(v)
# ax.plot_wireframe(x, y, z, color="r")


# ax.plot3D(points_x,points_y,points_z,'black', linewidth = 3.0)
# ax.set_aspect("equal")


# q0 = np.asarray([1.0,0.0,0.0,0.0])
# angular_velocity = np.asarray([0.0,0.0,np.pi/2])

# q0_0 = [q0[0]]
# q0_1 = [q0[1]]
# q0_2 = [q0[2]]
# q0_3 = [q0[3]]

# for i in range(numberIters):
#     q0 = equation(q0,T,angular_velocity)
#     q0_0.append(q0[0])
#     q0_1.append(q0[1])
#     q0_2.append(q0[2])
#     q0_3.append(q0[3])

# plt.plot(q0_0, range(numberIters+1))
# plt.plot(q0_1, range(numberIters+1))
# plt.plot(q0_2, range(numberIters+1))
# plt.plot(q0_3, range(numberIters+1))

# plt.plot(q0_0[0], 0.0, marker = 'x', color = 'k')
# plt.plot(q0_1[0], 0.0, marker = 'x', color = 'k')
# plt.plot(q0_2[0], 0.0, marker = 'x', color = 'k')
# plt.plot(q0_3[0], 0.0, marker = 'x', color = 'k')

# plt.plot(q0_0[-1], 1001.0, marker = 'x', color = 'green')
# plt.plot(q0_1[-1], 1001.0, marker = 'x', color = 'green')
# plt.plot(q0_2[-1], 1001.0, marker = 'x', color = 'green')
# plt.plot(q0_3[-1], 1001.0, marker = 'x', color = 'green')


# plt.show()
