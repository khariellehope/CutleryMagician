clc
clf
clear all

set(0,'DefaultFigureWindowStyle','docked');

% Model Jaco arm

jacoBase = transl(2.1, 1.5, 0.7)*trotz(pi);
qHomePose = [0 pi deg2rad(341) pi/2 2*pi 0];                  %Change joint angles accordingly
qTest = deg2rad([-0.1466,5.4629,3.6849,0,3.1416,0]); 
%Get robot arm 
robot = Jaco;                            %Calling the Jaco class
robot.GetJacoRobot();                     %Calling the 'GetJacoRobot' function from class - This function creates the model using dh parameters
robot.PlotAndColourRobot();               %Use ply files to model realistic Jaco Robot
%robot.model.base = jacoBase;              %Set base position
robot.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist'};
robot.model.plot(qHomePose, 'scale', robot.scale);      %Plot model         
hold on;