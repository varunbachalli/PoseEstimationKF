import numpy as np 


a_mag = 0.5
a_acc = 0.5
w_rotate_acc = np.asarray([0.0,0.0,1.0])
w_rotate_mag =  np.asarray([1.0,0.0,0.0])
w_initial_acc = np.asarray([0.0,0.0,1.0])
w_initial_mag = np.asarray([-1.0,0.0,0.0])


B_acc = a_acc * np.outer(w_rotate_acc,w_initial_acc)
B_mag = a_mag * np.outer(w_rotate_mag,w_initial_mag)
print(B_acc)
print(B_mag)

B_total = B_acc+B_mag

u,s,vh = np.linalg.svd(B_total)
M = np.diag(np.asarray([1,1,np.linalg.det(u)*np.linalg.det(vh)]))
# print(np.asarray([1,1,np.linalg.det(u)*np.linalg.det(vh)]))
print(M)
rotate = np.matmul(np.matmul(u,M),vh)
print(rotate)

print(np.dot(rotate,w_initial_mag))
# [a**2 + b**2 - c**2 - d**2  ,2*b*c - 2*a*d               ,2*b*d + 2*a*c],
#                     [2*b*c + 2*a*d              ,a**2 - b**2 + c**2 -d**2    ,2*c*d - 2*a*b],
#                     [2*b*d- 2*a*c               ,2*c*d + 2*a*b               ,a**2 - b**2 - c**2 +d**2]])


