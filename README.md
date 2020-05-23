# Pose estimation of Mobile phone using Quarternion based Extended Kalman Filter
> Work in progress

# Data Transfer from Mobile Phone to Computer

The data is transferred from Mobile phone to Computer over wifi through the `TCP/IP` protocol using Sockets. The computer is the server and the phone is the client.
check Server.cpp, Server.h for the implementation of the server. 

The data from the client is received as a formatted text containing, 
1. Stage of computation
    - Magnetometer Computation.
    - setting initial position.
    - Kalman Filter Execution.
2. type of the sensor : 
    - Accelerometer ( number 0)
    - Gyroscope (number 1)
    - Magnetometer (number 2)
3. x, y, z measurements
4. Time stamp of measurment.

In order to account for time for Execution of the Kalman Filter and the rate at which the data is received from the mobile phone, the server and filter run on seperate threads. The data is stored on the heap and the threads communicate using condition variables. 
___
## Extended Kalman Filter

The extended kalman filter is implemented by using the 4 components of the quaternion as states in the state vector, the measured angular velocity from the gyroscope as control input, the accelerometer and magnetometer reading as the measurment of angle. This will be explained in detail below. The general algorithm for the extended kalman filter used can be found in [EKF algorithm](http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/WELCH/kalman.2.html). 
___
## Relationship between quarternion and angular velocity
The state space equations chosen to represent the system is given as follows. 

q̇ = 1/2 Ω(ω)q where ω is the angular velocity. 

Ω(ω)  is given by

[0 -ω<sub>x</sub> -ω<sub>y</sub> -ω<sub>z</sub>]\
[ω<sub>x</sub> 0 ω<sub>z</sub>  -ω<sub>y</sub>]\
[ω<sub>y</sub> -ω<sub>z</sub> 0 ω<sub>x</sub>]\
[ω<sub>z</sub> ω<sub>y</sub> -ω<sub>x</sub> 0]

For details check [Integrating Quaternions](https://www.ashwinnarayan.com/post/how-to-integrate-quaternions).

This is a continuous first order state space equation. So to get the difference equation, I decided to use the Runge-Kutta 4 method.

Finally to get the updated Covariance matrix P, the Jacobian with respect to the states and Covariance matrices need to be found. This is fairly straight forward. The results can be found in KalmanFilter.cpp under GetJacobian_A and GetJacobian_B functions.
___
## SVD based solution to the Wahba problem.
The measurement of the quarternion is done by commparing it with the quaternion found from the accelerometer and magnetometer measurements. Given two vector observations, the optimal rotation matrix that represents their relative orientation can be found by solving the Wahba problem. One method to solve the wahba problem is the Singular Value Decomposition method. The magnetometer and accelerometer readings are first passed through a simple low pass filter to eliminate the noise, for better accuracy of the Wahba solution.

For more information [IIT Bombay Satellite Wiki](https://www.aero.iitb.ac.in/satelliteWiki/index.php/What_is_Estimation%3F_Estimate_What%3F), specifically for the algorithm used see [SVD method](https://en.wikipedia.org/wiki/Wahba%27s_problem). 
___
## Conversion of Rotation matrix to quarternion

Furthermore, once the Rotation matrix is found, it's necessary to convert it to a Quaternion in order to compare it with the EKF prediction. 

The algorithm to do this can be found in [Euclidean space blog](https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/). Don't forget to read the [Forum Discussion](https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/ethan.htm). 
___
## Animation 
The live animation of the phones orientation is done using OpenGL. The code can be seen in `OpenGLPlotter.cpp`. The vertex and fragment shaders can be found in `Basic.shader` . OpenGLPlotter recieves a quaternion as input, it converts it into a 4 x 4 transformation matrix, which is applied to the shape, which in this case is a cuboid. This transformation is used as the Model matrix in the model view projection system of matrices. 

___
## What is the Python code used for
The gyroscope , accelerometer and magnetometer readings, the resulting quaternions are stored in a CSV file, which is used to analyze the signals in the form of plots. Furthermore, the algorithm is also implemented in python, in order to be able to perform offline parameter tuning. 

Note: Make sure to change the location of where the file is saved in the CPP and python files respectively. Then run `main_file.py` to observe how the data changed. 
___
## Results: 

As can be seen from the results, the extended Kalman Filter gives the best possible solution. The Quaternion determined purely by the gyroscope is highly sensitive to the gyroscope noise. The shape is losely similar to the the estimated solution. The Quaternion determined purely by Wahba is accurate, however very noisy. The Kalman Filter provides the best possible estimate given the readings. If the process variance is smaller than the Measurement variance, then the Kalman Filter would more closely resemble the Gyroscope readings, and inversely it would follow the Wahba estimation better. Since the Wahba Solution is also passed through a low pass filter, I set the Measurement variance to be lower than the process variance. 

by changing the 
```python
k.setQ(1)
k.setR(0.1)
```

in the main_file.py, the change in the KalmanFilter estimation can be seen. 


![W Values ](/Results/w.png)

![X Values ](/Results/x.png)

![Y Values ](/Results/y.png)

![Z Values ](/Results/z.png)

![Video Demonstration](https://youtu.be/qK9Tyd0Xv1k)

___
## Issues and further work

The biggest issue with the algorithm is that the spatial rotation represented by q and -q are equal. Therefore, while comparing the result for the Wahba Problem it is important to compare it with the correct quaternion. The method i used to remedy this is to take the dot product of the measurement and the prediction. If the result < 0.0 then I compare the prediction with the the negative of the measurment. In an ideal situation, the prediction and the correction quaternions will have the same values and the norm of the quaternions needs to be 1. Therefore, if the dot product is =1 then both the quaternions are pointing in the same direction, else they're pointing in opposite directions. 

___
# Extra
## Magnetometer Calibration

A magnetometer measures the direction and strength of the magnetic field at a particular location. Standard magnetometer noise is in the form of Hard Iron and Soft Iron errors. In an ideal situation where there is no interfering magnetic field, the magnetometer should point towards true north in the earth fixed coordinate system, but measured with respect to the phone fixed coordinate system.  If magnetometer measurements are taken as the sensor is rotated through all possible orientations, the measurements should lie on a sphere centered on the origin. However due to magnetic interference, from hard and soft-iron sources, the sphere turns into an ellipsoid with an offset center. Therefore, assuming that the interference is always constant at a given position in space, it is possible to find a map between the offset ellipsoid and a unit circle. In order to do this, **N** raw magnetometer readings are obtained from all possible orientations of the phone. These **N** readings are used to fit an ellipsoid using the least squares method. Finally the corresponding bias, rotation and scaling matrix is computed.
For more information check 
___
### General Equation of an ellipsoid with offset

The general equation of an ellipsoid is given by 

ax̂ <sup>2</sup> +  bŷ<sup>2</sup> + cẑ<sup>2</sup> + 2dx̂ŷ + 2dx̂ẑ + 2fŷẑ = 1

where 
- x̂ = x - x'
- ŷ = y - y'
- ẑ = z - z'

Therefore, the bias is calculated using the mean x', y' and z' of the **N** readings. Each of the readings is then subtracted by the bias, and solving the least squares problem, the values for a,b,c,d,e,f are found.

The least squares problem is solved using CUDA. Not for any special purpose, it was only to learn the basics of gpgpu programming using CUDA C. see Matrix_multiplication.cu and .cuh for more details.

In matrix form the equation of the ellipsoid is given by A**x̂** = **1**. Where A is symmetric and given by

    [a d e]
    [d b f] 
	[e f c]

As can be seen in this paper, [Magnetometer Calibration Paper](https://www.nxp.com/docs/en/application-note/AN4246.pdf), it is necessary to find A<sup>1/2</sup> which represents the rotation and scaling matrix.

This is done by using diagonalization method, where you find the 

Any matrix (A) can be written as A = VDV<sup>−1</sup> where D and V are the eigen value and eigen vector matrices respectively. An eigen vector matrix, is a diagonal matrix with the diagonals containing the eigen values.

Finally A<sup>1/2</sup> can be calculated by VD<sup>1/2</sup>V<sup>−1</sup> where D<sup>1/2</sup> is calculated by taking the square root of each of it's elements. The Eigen values of an ellipse are always positive.

In order to find the eigen values and eigen vector matrices, the recursive Jacobi method is used. details can be found in 
[Iterative methods for eigenvalue problem](https://www.cmi.ac.in/~ksutar/NLA2013/iterativemethods.pdf). The implementation can be seen in the Magnetometer_Calibration.cpp file. Again, it was implemented just for learning purposes, the Eigen library for C++ has a method to solve this. In order to get the corrected value the following equation needs to be evaluated.

Unfortunately this doesn't work as well as expected. Besides, the Magnetometer readings from Android come precalibrated. Hence it's skipped.

x<sub>calibrated</sub> = A<sup>1/2</sup> ( x - x')


[Magnetometer Calibration Paper](https://www.nxp.com/docs/en/application-note/AN4246.pdf) and [magcal operation in Matlab](https://de.mathworks.com/help/fusion/ref/magcal.html)