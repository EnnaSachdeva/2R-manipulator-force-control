%% 2R manipulator Force control using feedback linearization

% This script runs the manipualtor to reach a given final point within its workspace, 
% by calculating the error at each time step and then applying controls to reach at that particular point.

% 04-03-2017
% Enna Sachdeva

close all;
clear all;
clc;

%% Initial and final configurations
thInit=[0 0];    % initial points 

thFin=[pi/2 pi/2];    %final points

x0=[0 0 thInit 0 0 0 0];  %[intTheta1  intTheta2 Theta1 Theta2 ThetaDot1 ThetaDot2 F1 F2]
%states initial values
tSpan=[0 20]; %time span

%% Manipulator Specifications
M=[1,1];   % link mass
L=[1,1];   % link length
params=[M,L];

%% Manipulator Dynamics and control using feedback linearization
% PID control Parameters for joint1 and joint2

% Joint 1
Kp1=15;
Kd1=7;
Ki1=10;

% Joint 2
Kp2=15;
Kd2=10;
Ki2=10;

Kpid=[Kp1 Kd1 Ki1 Kp2 Kd2 Ki2];

% ODE solver
func=@(t,x) dynamicsNcontrol(t,x,thFin,params,Kpid);
[tstep,angleParams] = ode45(func,tSpan,x0); 


%% Output angles and Force
theta=[angleParams(:,3),angleParams(:,4)]; 

F1=diff(angleParams(:,7))./diff(tstep);  % calculates the approximate derivatives by calculating 
F2=diff(angleParams(:,8))./diff(tstep);  % the approximate differences of the adjacent elements
time=0:(tstep(end)/(length(F1)-1)):tstep(end);



%% Applying forward kinematics
ForwKin = forwardKinematics(params, theta);
x1=ForwKin(:,1); 
y1=ForwKin(:,2); 
x2=ForwKin(:,3); 
y2=ForwKin(:,4); 

err1=(thFin(1)-theta(1))*180/pi;
err2=(thFin(2)-theta(2))*180/pi;

% angle error plot
figure(01)
plot(tstep,err1,'b.',tstep,err2,'r.')
axis([0 tstep(end) -5 200]);
grid 
title('Angles error')
ylabel('Theta error (in deg)')
xlabel('time (in sec)')
legend('Theta1', 'Theta2');

% torque1 plot
figure(02)
plot(time,F1,'b',time,F2,'r')
grid
title('Torque of joints');
ylabel('Joints torque (in N)');
xlabel('time (in sec)');
legend('Joint1', 'Joint2');
hold off

%% Plots 

h=figure(03);
for i=1:length(tstep)-1
    figure(h)
    plot(0,0,'blacko',[x1(i) x2(i)],[y1(i) y2(i)],'b*',[0 x1(i)],[0 y1(i)],'r',[x1(i) x2(i)],[y1(i) y2(i)],'r')
    title('Trajectory of a 2DOF Manipulator')
    xlabel('x')
    ylabel('y')
    axis([-4 4 -4 4]);
    grid
    pause(0.001)
end

drawnow;
