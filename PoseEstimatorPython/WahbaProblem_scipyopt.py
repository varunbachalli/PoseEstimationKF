import numpy as np 
from scipy.optimize import minimize


a_mag = 0.5
a_acc = 0.5
w_rotate_acc = np.asarray([0.0,0.0,1.0])
w_rotate_mag = np.asarray([1.0,0.0,0.0])
w_initial_acc = np.asarray([0.0,0.0,1.0])
w_initial_mag = np.asarray([-1.0,0.0,0.0])

def norm(a):
    result = 0.0
    for i in a:
        result+= i**2
    result = np.sqrt(result)
    return result

def Wahba(x,w_rotate,w_initial):
    a = x[0]/norm(x)
    b = x[1]/norm(x)
    c = x[2]/norm(x)
    d = x[3]/norm(x)
    equation_1 =  w_rotate[0] - (a**2 + b**2 - c**2 - d**2) * w_initial[0]  + (2*b*c - 2*a*d)*w_initial[1] + (2*b*d + 2*a*c)*w_initial[2]
    equation_2 =  w_rotate[1] - (2*b*c + 2*a*d)*w_initial[0] + (a**2 - b**2 + c**2 -d**2)*w_initial[1]+(2*c*d - 2*a*b)*w_initial[2]
    equation_3 =  w_rotate[2] - (2*b*d- 2*a*c)*w_initial[0] + (2*c*d + 2*a*b)*w_initial[1] + (a**2 - b**2 - c**2 +d**2)*w_initial[2]

    return (equation_1)**2 +(equation_2)**2 +(equation_3)**2
    

def objective(x):
    global w_rotate_acc, w_rotate_mag,w_initial_acc,w_initial_mag
    eq_acc = Wahba(x,w_rotate_acc,w_initial_acc)
    eq_mag = Wahba(x,w_rotate_mag,w_initial_mag)
    return eq_acc + eq_mag


delta = 0.01
x0 = np.asarray([np.cos((np.pi+delta)/2),0.0,0.0,np.sin((np.pi+delta)/2)])
sol = minimize(objective,x0, method= 'SLSQP', options = {'disp':True,'ftol': 1e-20})

xopt = sol.x

print(xopt)

print(Wahba(x0,w_rotate_acc,w_initial_acc))
print(Wahba(x0,w_rotate_mag,w_initial_mag))


# [a**2 + b**2 - c**2 - d**2  ,2*b*c - 2*a*d               ,2*b*d + 2*a*c],
#                     [2*b*c + 2*a*d              ,a**2 - b**2 + c**2 -d**2    ,2*c*d - 2*a*b],
#                     [2*b*d- 2*a*c               ,2*c*d + 2*a*b               ,a**2 - b**2 - c**2 +d**2]])


