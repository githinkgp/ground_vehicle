#!/usr/bin/env python


from scipy.io import loadmat
from math import *
import numpy as np
import math
import matplotlib.pyplot as plt


class LvlSetGenerator(object):
	"""class for level set of reachable set generator, initialize with data height. read data from matlab data.mat"""
	def __init__(self, hight): ## input argument is string "high" "med" "low"
		name = 'data_'+hight #choose wich data set to use 
		data = loadmat('data_GV.mat')
		self.data_0 = data[name]  #load data
		self.sx, self.sy, self.st = self.data_0.shape #shape of contor sets
		#print self.data_0.shape
		self.theta_int = 2*pi/self.st  #incremental theta


	"""generate indices where for level set, out put format (row,col) list"""
	def CalculateLevelSet(self,theta):
		level_val = 0.0
		theta_ind =int(floor(theta/self.theta_int))
		if theta_ind == 100:
			print "INDEX"
		contor = self.data_0
		level_setX= [];
		level_setY= [];
		#print self.sx, self.sy
		for i in range(self.sx-1):
			for j in range(self.sy-1):
				if (contor[i][j][theta_ind] <= level_val and contor[i+1][j][theta_ind] >= level_val) or (contor[i][j][theta_ind] >= level_val and contor[i+1][j][theta_ind] <= level_val) or (contor[i][j][theta_ind] <= level_val and contor[i][j+1][theta_ind] >= level_val) or (contor[i][j][theta_ind] >= level_val and contor[i][j+1][theta_ind] <= level_val):
					level_setX.append((i-50)*0.1)
					level_setY.append((j-50)*0.1)

				#if (contor[i][j][theta_ind] <= level_val):
				#	print "minus"
				
		return [level_setX,level_setY]



"""example code"""
#lvls = LvlSetGenerator('low')
#X,Y = lvls.CalculateLevelSet(1)
#plt.scatter(X,Y)
#plt.show()
#print X
