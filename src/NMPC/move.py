#!/usr/bin/env python
import math
import numpy as np
from array import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def move(x,u,dt):
    x_new=x[0]+dt*u[0]*math.cos(x[2])
    y_new=x[1]+dt*u[0]*math.sin(x[2])
    t_new=x[2]+dt*u[1]
    
    return x_new, y_new, t_new
