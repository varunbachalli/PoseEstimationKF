import matplotlib.pyplot as plt
import numpy as np


def plot:
	x = np.linspace(0,100.0)
	y = 10*x + 100
	plt.figure()
	plt.plot(x,y)
	plt.show()