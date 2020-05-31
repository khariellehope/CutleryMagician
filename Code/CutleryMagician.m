clc
clf
clear all

set(0,'DefaultFigureWindowStyle','docked');

% Model Jaco arm
scale = 0.1;
jacoBase = transl(1.25, 1.25,0.25);
qHomePose = [pi/180,0,0,0,0,0];                  %Change joint angles accordingly

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

%Using environment function:
xOffset = 0;
yOffset = 0;
zOffset = -0.5;
partMesh = Environment('kitchen.ply', xOffset, yOffset, zOffset);
benchMesh_h = partMesh;
hold on;

%Import containers
%Locations of each container (hard coded):
containerOneLoc = transl(0.7, 1, 0.2);
containerTwoLoc = transl(0.9, 1, 0.2);
containerThreeLoc = transl(1.1, 1, 0.2);

partMesh = Environment('Container1.ply', containerOneLoc(1,4), containerOneLoc(2,4), containerOneLoc(3,4));
containerOneMesh_h = partMesh;
hold on;

partMesh = Environment('Container2.ply', containerTwoLoc(1,4), containerTwoLoc(2,4), containerTwoLoc(3,4));
containerTwoMesh_h = partMesh;
hold on;

partMesh = Environment('Container3.ply', containerThreeLoc(1,4), containerThreeLoc(2,4), containerThreeLoc(3,4));
containerThreeMesh_h = partMesh;
hold on;

%Floor
x = [2 -2.5; 2 -2.5];
y = [2 2; -2.5 -2.5];
z = [-0.5 -0.5; -0.5 -0.5];

floor_h = background('floor.jpeg', x, y, z);

%Window Wall
x = [1.1 -2.5; 1.1 -2.5];
y = [2 2; 2 2];
z = [1 1; -0.6 -0.6]; 
window_h = background('windowWall.jpg', x, y, z);

%% Import Cutlery

spoonLoc = transl(1.3, 1, 0.25);
forkLoc = transl(1.4, 1, 0.25);
knifeLoc = transl(1.5, 1, 0.25);

partMesh = Environment('Spoon.ply', spoonLoc(1,4), spoonLoc(2,4), spoonLoc(3,4));
spoonMesh_h = partMesh; 

partMesh = Environment('Fork.ply', forkLoc(1,4), forkLoc(2,4), forkLoc(3,4));
forkMesh_h = partMesh;

partMesh = Environment('Knife.ply', knifeLoc(1,4), knifeLoc(2,4), knifeLoc(3,4));
knifeMesh_h = partMesh;



%% Collision avoidance?
%Check for collision with barrier/box or container(s)??

%% Sensor data?

%% Movement



