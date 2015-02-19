import os,sys
import pylab
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import math 
# setup font
font =  {'family' : 'sans-serif',
         'color'  : 'black',
         'weight' : 'normal',
         'size'   : 16,
        }

# step size (constant value)
DT = 0.001

# load data
y1 = pylab.loadtxt("./track_sinusoidal.desired")
y2 = pylab.loadtxt("./track_sinusoidal.state")
t = np.linspace(0,y1[:,0].size*DT,y1[:,0].size)

# setup the figure
fig = plt.figure()

# plot data
plt.plot(t,y1[:,0],'k' ,label='Desired (joint 1)')
plt.plot(t,y2[:,0],'r' ,label='Actual (joint 1)')

# add titles, labels, and legend
plt.title('Joint 1 Tracking Response sinusoidal trajectory', fontdict=font)
plt.xlabel('Time', fontdict=font)
plt.ylabel('Joint angle', fontdict=font)
plt.legend(loc=1, shadow=True)
plt.axis([0,10,-math.pi, math.pi])
# show the 
plt.savefig('plot_track_sinusoidal.png')
plt.show()


