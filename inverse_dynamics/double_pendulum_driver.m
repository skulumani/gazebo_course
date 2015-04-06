function [] = double_pendulum_driver()
clear all
clc
close all

movie = false;
duration = 10;
fps = 30;

% constants

constants.g = 9.81;
constants.m1 = 2;
constants.m2 = 1;
constants.l1 = 1;
constants.l2 = 1;

constants.overshoot = 0.1; % percent overshoot
constants.settling = 1; % settling time in sec

nframes=duration*fps;
tspan = linspace(0,duration,nframes);
initial_state = [0;0;0;0;0;0]; % extra 2 states for integral action of each joint
[t,state] = ode45(@(t,state)double_pendulum_ode(t,state,constants),tspan,initial_state);

% calculate the control input
control = zeros(length(t),2);
for ii = 1:length(t)
    [~, u] = double_pendulum_ode(t(ii),state(ii,:)',constants);
    control(ii,:) = u';
end

t1 = state(:,1);
t2 = state(:,2);
td1 = state(:,3);
td2 = state(:,4);

h=plot(0,0,'MarkerSize',30,'Marker','.','LineWidth',2,'Color','b');
range=1.1*(constants.l1+constants.l2);
axis([-range range -range range]);
axis square;
grid on
title('Double Pendulum')
xlabel('X Axis')
ylabel('Y Axis')
set(gca,'nextplot','replacechildren');

for i=1:length(t1)-1
    if (ishandle(h)==1)
        r1 = constants.l1*[sin(t1(i));-cos(t1(i))];
        r2 = r1 + constants.l2*[sin(t1(i)+t2(i));-cos(t1(i)+t2(i))];
        Xcoord=[0,r1(1),r2(1)];
        Ycoord=[0,r1(2),r2(2)];

        set(h,'XData',Xcoord,'YData',Ycoord);
        drawnow;
        if movie ==true
            
            F(i) = getframe;
        end
        if movie==false
            pause(t(i+1)-t(i));
        end
    end
end

figure
grid on;hold on
plot(t,t1*180/pi, t,t2*180/pi)
legend('Joint 1','Joint 2')
xlabel('Time (sec)')
ylabel('Angle (deg)')

figure
grid on;hold on
plot(t,control(:,1), t,control(:,2))
legend('Joint 1','Joint 2')
xlabel('Time (sec)')
ylabel('Torque (Nm)')

if movie==true
    movie2avi(F,'doublePendulumAnimation','compression','none','fps',fps)
end

% 16 Jan 15 - double pendulum dynamics

function [state_dot, u] = double_pendulum_ode(t,state,constants)

t1 = state(1);
t2 = state(2);
td1 = state(3);
td2 = state(4);
int_error = state(5:6,1);

m1 = constants.m1;
m2 = constants.m2;
l1 = constants.l1;
l2 = constants.l2;
g = constants.g;
overshoot = constants.overshoot;
settling = constants.settling;

% theta1 zero is downwards theta 2 zero is aligned with first link
M = zeros(2,2);
M(1,1) = (m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(t2);
M(1,2) = m2*l2^2 + m2*l1*l2*cos(t2);
M(2,1) = m2*l2^2 + m2*l1*l2*cos(t2);
M(2,2) = m2*l2^2;

V = zeros(2,2);
V(1,2) = -m2*l1*l2*(2*td1 + td2)*sin(t2);
V(2,1) = m2*l1*l2*td1*sin(t2);

G = zeros(2,1);
G(1,1) = g*((m1+m2)*l1*sin(t1) + m2*l2*sin(t1 + t2));
G(2,1) = g*m2*l2*sin(t1+t2);

B = eye(2,2);

zeta = -log(overshoot/100)/sqrt(pi^2+log(overshoot/100)^2);
wn = 4/settling/zeta;

E_dot = [0;0]-[td1;td2];
E = [3*pi/4;pi/4]-[t1;t2];

Kv = diag(2*zeta*wn*[1 1]);
Kp = diag(wn^2*[1 1]);

u = V*[td1;td2] + G + M*(Kv*E_dot + Kp*E); % feedback linearization

accel = M\( -V*[td1;td2] - G + B*u);

state_dot = [td1;td2;accel;E+int_error];