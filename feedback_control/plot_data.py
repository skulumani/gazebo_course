# python script to plot double pendulum data
import numpy as np
import matplotlib.pyplot as plt
import math
# load the data

# radian to degree conversion
rad2deg = 180/math.pi
deg2rad = math.pi/180
tol = 0.001

# load the desired state and theta 1 state and velocity 
# theta1_des = np.loadtxt("./PID_hold.desired")
# theta1_state = np.loadtxt("./PID_hold.state")
theta1_des = np.loadtxt("./track_sinusoidal.desired")
theta1_state = np.loadtxt("./track_sinusoidal.state")
# step size of each data point output from gazebo
# this is defined in the model file
DT = 0.001
# create the time array
t = np.linspace(0,theta1_state[:,0].size*DT,theta1_state[:,0].size)

# plotting limits
xmin = 0
xmax = 10
ymin = -90.0
ymax = 90.0

# calculate steady state % error bounds
error = 0.05 # error in radians 

steady_state = theta1_state[-1,0] # final value of response
set_point = theta1_des[-1,0] # final value of desired state

pos_error_band = set_point + error 
neg_error_band = set_point - error 

# calculate the 0% to 100% rise time of the position response
# find time when response first hits the steady state value
rise_index = np.nonzero(np.abs(theta1_state[:,0]-steady_state) < tol)[0][0]
#rise_index = 1
rise_time = t[rise_index]

rise_time_x = rise_time
rise_time_y = np.abs((theta1_state[rise_index,0]*rad2deg) - ymin)/(ymax-ymin)

# calculate the peak time. time to reach the first maximum value
maximum = theta1_state[:,0].max() # max of all data but not necessarily the first
peak_index = np.nonzero(np.abs(theta1_state[:,0] - maximum) < tol)[0][0]
peak_time = t[peak_index]

peak_time_x = peak_time
peak_time_y = np.abs((theta1_state[peak_index,0]*rad2deg) - ymin)/(ymax-ymin)

# calculate percent overshoot
overshoot = (maximum - steady_state) / np.abs(steady_state - ymin) * 100
overshoot_x = np.abs(peak_time - xmin) / (xmax - xmin)

# calculate settling time
# the last time the state is outside the error band limits
settling_time_index = np.nonzero(np.abs(theta1_state[:,0] - steady_state) > error)[-1][-1]
settling_time = t[settling_time_index]

settling_time_x = settling_time
settling_time_y = np.abs((theta1_state[settling_time_index,0]*rad2deg) - ymin)/(ymax-ymin)
# steady state error
steady_state_error = np.abs(steady_state - set_point)

print "0-100 Rise Time = %f sec" % rise_time_x
print "Maximum = %f deg at %f sec" % ( maximum * rad2deg,peak_time_x)
print "Percent Overshoot = %f at %f sec" % (overshoot, peak_time)
print "Overshoot = %f rad" % (maximum - steady_state)
print "Settling time = %f sec" % (settling_time)
print "Steady State error = %f deg" % (steady_state_error * rad2deg)

# remember to convert to degrees when actually plotting
#########################################
# plot the position of the first joint
plt.figure(1)
plt.plot(t,theta1_state[:,0]*rad2deg,'r', label = 'Actual')
plt.plot(t,theta1_des[:,0]*rad2deg,'k', label = 'Desired')

# set the axis of the plot
plt.axis([xmin,xmax,ymin, ymax])
# axis labels
plt.title('Joint 1 Tracking Response')
plt.xlabel('Time (sec)')
plt.ylabel('Joint angle (deg)')

plt.legend(loc=4, shadow=True)

# # plot set point error bounds
# plt.axhspan(neg_error_band*rad2deg,pos_error_band*rad2deg, facecolor='0.5', alpha=0.25)
# # rise time plotting
# plt.axvline(x=rise_time,ymin = 0, ymax = rise_time_y, color='k',linestyle='--')
# plt.annotate('T_r', xy=(rise_time, -60))
# # peak time plotting
# plt.axvline(x=peak_time,ymin = 0, ymax = peak_time_y, color='k',linestyle='--')
# plt.annotate('T_p', xy=(peak_time, -45))
# # plot overshoot
# plt.axhline(y=maximum*rad2deg, xmin=0, xmax=overshoot_x,color='k',linestyle='--')
# plt.annotate('OS', xy=(peak_time, maximum))

# # plot settling time
# plt.axvline(x=settling_time_x, ymin=0, ymax=settling_time_y,color='k',linestyle='--')
# plt.annotate('T_s', xy=(settling_time_x, -60))
# show the plot
plt.savefig('tracking_sinusoidal_1.png')
plt.show()

# plot the second joint
plt.figure(2)
# plot the position of the second joint
plt.plot(t,theta1_state[:,1]*rad2deg,'r', label = 'Actual')
plt.plot(t,theta1_des[:,1]*rad2deg,'k', label = 'Desired')

# set the axis of the plot
plt.axis([xmin,xmax,ymin, ymax])
# axis labels
plt.title('Joint 2 Tracking Response')
plt.xlabel('Time (sec)')
plt.ylabel('Joint angle (deg)')

plt.legend(loc=4, shadow=True)

# # plot set point error bounds
# plt.axhspan(neg_error_band*rad2deg,pos_error_band*rad2deg, facecolor='0.5', alpha=0.25)
# # rise time plotting
# plt.axvline(x=rise_time,ymin = 0, ymax = rise_time_y, color='k',linestyle='--')
# plt.annotate('T_r', xy=(rise_time, -60))
# # peak time plotting
# plt.axvline(x=peak_time,ymin = 0, ymax = peak_time_y, color='k',linestyle='--')
# plt.annotate('T_p', xy=(peak_time, -45))
# # plot overshoot
# plt.axhline(y=maximum*rad2deg, xmin=0, xmax=overshoot_x,color='k',linestyle='--')
# plt.annotate('OS', xy=(peak_time, maximum))

# # plot settling time
# plt.axvline(x=settling_time_x, ymin=0, ymax=settling_time_y,color='k',linestyle='--')
# plt.annotate('T_s', xy=(settling_time_x, -60))
# show the plot
plt.savefig('tracking_sinusoidal_2.png')
plt.show()
