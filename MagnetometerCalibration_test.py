# import cv2
import numpy as np
import math
# import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sklearn import linear_model, datasets

def rotate(point,rotation_in_angles):# XZX rotiations check wikipedia euler angles
    c1 = np.cos(rotation_in_angles[0])
    c2 = np.cos(rotation_in_angles[1])
    c3 = np.cos(rotation_in_angles[2])
    s1 = np.sin(rotation_in_angles[0])
    s2 = np.sin(rotation_in_angles[1])
    s3 = np.sin(rotation_in_angles[2])

    Rotation_matrix = np.asarray([[c2, -1*c3*s2, s2*s3],
                                [c1*s2, c1*c2*c3 - s1*s3, -c3*s1 - c1*c2*s3],
                                [s1*s2, c1*s3 + c2*c3*s1, c1*c3 - c2*s1*s3]])

    
    return np.dot(Rotation_matrix,point)



def generate_data_polar(elips_equation,bias, noise, rotation_in_angles, number_of_elements): 
    theta = np.random.uniform(0, np.pi, number_of_elements)
    phi = np.random.uniform(0, 2*np.pi, number_of_elements)
    x = elips_equation[0] * np.multiply(np.sin(theta), np.cos(phi)) 
    y = elips_equation[1] * np.multiply(np.sin(theta), np.sin(phi)) 
    z = elips_equation[2] * np.cos(theta)
    var = np.asarray([x,y,z])
    var = np.transpose(var)
    for i in range(len(var)):
        var[i] = rotate(var[i],rotation_in_angles)
    # here var = (1000, 3)
    # here it's possible to add bias by var = var + bias
    # add noise
    var = var + np.random.uniform(-1*noise, noise, var.shape)
    # add bias
    bias = np.asarray(bias)
    var = var + bias
    return var

def largestOffDiagonalElement(A):
    k = 0
    l = 0
    max_value = float('-inf')
    for i in range(A.shape[0]):
        for j in range(i+1, A.shape[1]):
            if(A[i][j] > max_value):
                max_value = A[i][j]
                k = i
                l = j
    return k,l
def eigvalues_and_vector(A): # only diagonal matrices
    D = A
    S = np.eye(A.shape[0])
    isdiagonal = False
    while(~isdiagonal):
        i,j = largestOffDiagonalElement(D)
        theta = 0.0
        if(D[i][i] - D[j][j] == 0.0):
            if(D[i][i]>0):
                theta = np.pi/4
            else:
                theta = -np.pi/4

        else:
            theta = 0.5* np.arctan(2*D[i][j]/(D[i][i] - D[j][j]))

        if(np.cos(theta)<= 1 + 10**-4 and np.cos(theta) >= 1 - 10**-4):
            break
        S1 = np.eye(len(A))
        S1[i][i] = np.cos(theta)
        S1[j][j] = np.cos(theta)
        S1[i][j] = -np.sin(theta)
        S1[j][i] = np.sin(theta)

        D = np.dot(np.transpose(S1),np.dot(D,S1))
        S = np.dot(S,S1)

    eig_vals = []
    for i in range(len(D)):
        eig_vals.append(D[i][i])
    D = np.asarray(eig_vals)*np.eye(len(eig_vals))
    return D,S

def plotpoints(variables, fig, ax, m, color):
    x = []
    y = []
    z = []

    for var in variables:
        x.append(var[0])
        y.append(var[1])
        z.append(var[2])

    
    ax.scatter(x, y, z, c=color, marker=m)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    
    plt.figure()
    plt.plot(x,y,'rx')
    plt.axis('equal')
    # plt.savefig("xy.jpg")
    plt.figure()
    plt.plot(x,z,'rx')
    plt.axis('equal')
    # plt.savefig("xz.jpg")
    plt.figure()
    plt.plot(z,y,'rx')
    plt.axis('equal')
    # plt.savefig("zy.jpg")
    

def compute_R(variables):
    A = []
    for var in variables:
        a = [var[0]**2,var[1]**2,var[2]**2, 2*var[0]*var[1], 2*var[1]*var[2], 2*var[2]*var[0]]
        A.append(a)
    B = np.ones(number_of_elements)
    A = np.asarray(A)
    [a,b,c,d,e,f] = np.dot(np.linalg.pinv(A),B)
    R = np.asarray([[a,d,e],[d,b,f],[e,f,c]])
    return R


def computeBias(variables,number_of_elements):
    x = 0.0
    y = 0.0
    z = 0.0

    for var in variables:
        x+=var[0]
        y+=var[1]
        z+=var[2]
    
    x/=number_of_elements
    y/=number_of_elements
    z/=number_of_elements

    return [x,y,z]

def checkOrthogonal(A):
    k = np.dot(np.transpose(A),A) - np.eye(len(A))
    eps = 10**-3*np.ones(k.shape)
    a = k <= eps 
    b = k>= -1*eps
    return a.any() or b.any()
    

if __name__ == "__main__":

    # orthogonality_checker = []
    # totalError = []
    # number_elem = []
    # eig_is_positive = []

    error = 0.0
    noise_amplitude = 0.0
    number_of_elements = 1000
    rotation_in_angles = np.asarray([30,45,0])*np.pi/180 #XZX rotiations 
    variables = generate_data_polar([5,3,5],[2,3,5],noise_amplitude,rotation_in_angles,number_of_elements)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    marker = 'o'
    color = 'g'

    #plotpoints(variables,fig,ax,marker,color)

    bias = np.asarray(computeBias(variables,number_of_elements))
    variables = variables - bias
    error += abs(np.asarray([2,3,5]) - bias)
    R = compute_R(variables)
    eigvals , eigvector = eigvalues_and_vector(R)
    
    
    print("matrix\n",R)
    print("eigenvalues\n",eigvals)
    print("eigvector\n",eigvector)

    W = np.dot(np.dot(eigvector,np.sqrt(eigvals)),np.transpose(eigvector))

    transformed_variables  = []
    for var in variables:
        transformed_variables.append(np.dot(W,var))

    marker = 'x'
    color = 'r'

    # plotpoints(transformed_variables,fig,ax,marker,color)
    # plt.show()


    # matrix = np.random.rand(3,3)
    # eigvals, eigvector = eigvalues_and_vector(matrix)





    
    # if(checkOrthogonal(eigvector) == False):
    #     print("this wasn't ortho hence breakk")
    #     break
    # elif(i == 999):
    #     print("all ortho")

    
    
    # plt.figure()
    # plt.plot(number_elem,totalError)
    # plt.show()


        # var = np.asarray(variables)
        # var = var**2
        # y = np.ones(var.shape[0])
        # x = np.linalg.lstsq(var, y, rcond=None)[0]
        # x = [np.sqrt(1/abs(x_)) for x_ in x]
        # print(x)
        # print([10,100,20])
        # print(x)

    #plotpoints(variables)
    #fit using least squares and RANSAC

