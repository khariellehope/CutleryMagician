clc
clf
clear all

set(0,'DefaultFigureWindowStyle','docked');

%% Model Jaco arm
scale = 0.1;
jacoBase = transl(-0.2, 0.4,0.2);
qHomePose = zeros(1,6);                  %Change joint angles accordingly

%Get robot arm 
robot = Jaco;                            %Calling the Jaco class
robot.GetJacoRobot();                     %Calling the 'GetJacoRobot' function from class - This function creates the model using dh parameters
robot.PlotAndColourRobot();               %Use ply files to model realistic Jaco Robot
robot.model.base = jacoBase;              %Set base position
robot.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist'};
robot.model.plot(qHomePose, 'scale', scale, 'workspace', robot.workspace);      %Plot model         
hold on;

% use switch case for determining movement of robot/end effector to box
% depending on which cutlery was picked up


%%
% Import Environment

%Kitchen Bench

% [kitf, kitv, kitdata] = plyread('kitchen.ply','tri');
% kitVertexCount = size(kitv,1);
% midPoint = sum(kitv)/kitVertexCount;
% kitVerts = kitv - repmat(midPoint, kitVertexCount, 1);
% kitpose = eye(4);
% zOffset = 0;
% xOffset = 0;
% yOffset = 0;
% 
% vertexColours = [kitdata.vertex.red, kitdata.vertex.green, kitdata.vertex.blue]/255;
% kitMesh_h = trisurf(kitf, kitVerts(:,1) + xOffset, kitVerts(:,2) + yOffset, kitVerts(:,3) + zOffset ...
%     , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
% hold on;

%Using environment function:
xOffset = 0;
yOffset = 0;
zOffset = -0.5;
partMesh = Environment('kitchen.ply', xOffset, yOffset, zOffset);
benchMesh_h = partMesh;
hold on;

%Floor
x = [2 -2.5; 2 -2.5];
y = [2 2; -2.5 -2.5];
z = [-0.6 -0.6; -0.6 -0.6];

floor_h = background('floor.jpeg', x, y, z);

%Window Wall
x = [1.1 -2.5; 1.1 -2.5];
y = [2 2; 2 2];
z = [1 1; -0.6 -0.6]; 
window_h = background('windowWall.jpg', x, y, z);

%% Collision avoidance?


%% Sensor data?

