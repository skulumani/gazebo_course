import os,sys
import pylab
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines

# setup font
font =  {'family' : 'sans-serif',
         'color'  : 'black',
         'weight' : 'normal',
         'size'   : 16,
        }

# step size (constant value)
DT = 0.001

# load data
y = pylab.loadtxt("./track_sinusoidal.torque")
t = np.linspace(0,y[:,0].size*DT,y[:,0].size)

# setup the figure
fig = plt.figure()

# plot data
plt.plot(t,y[:,1],'k' ,label='PD control torque (joint 1)')
plt.plot(t,y[:,0],'r' ,label='inverse dynamics torque (joint 1)')

# add titles, labels, and legend
plt.title('Motor torques for PD control vs. inverse dynamics control for track sinusoidal task', fontdict=font)
plt.xlabel('Time', fontdict=font)
plt.ylabel('Torque', fontdict=font)
plt.legend(loc=4, shadow=True)

# save the plot
plt.savefig('plot_track_sinusoidal_torques.png')

