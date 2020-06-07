clc
clf
clear all

set(0,'DefaultFigureWindowStyle','docked');

% Model Jaco arm

jacoBase = transl(1.5, 1,0.25)*trotz(pi);
qHomePose = [0 pi/2 pi/2 0 pi 0];                  %Change joint angles accordingly
qTest = deg2rad([-0.1466,5.4629,3.6849,0,3.1416,0]); 
%Get robot arm 
robot = Jaco;                            %Calling the Jaco class
robot.GetJacoRobot();                     %Calling the 'GetJacoRobot' function from class - This function creates the model using dh parameters
robot.PlotAndColourRobot();               %Use ply files to model realistic Jaco Robot
robot.model.base = jacoBase;              %Set base position
robot.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist'};
robot.model.plot(qTest, 'scale', robot.scale, 'workspace', robot.workspace);      %Plot model         
hold on;

% Import Environment

%Kitchen Bench

%Using environment function:
xOffset = 0;
yOffset = 0;
zOffset = -0.5;
[partMesh,f,v] = Environment('kitchen.ply', xOffset, yOffset, zOffset);
benchMesh_h = partMesh;
benchFaces = f;
benchVertices = v;
% benchFN = benchMesh_h.FaceNormals;
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


%
%Import containers
%Locations of each container (hard coded):
containerOneLoc = transl(0.9, 1, 0.2);
containerTwoLoc = transl(1, 1, 0.2);
containerThreeLoc = transl(1.1, 1, 0.2);

%ContainerOneLoc = transl(1.41, 1, 0.25);

[partMesh, f, v] = Environment('Container1.ply', containerOneLoc(1,4), containerOneLoc(2,4), containerOneLoc(3,4));
containerOneMesh_h = partMesh;
containerOneFaces = f;
containerOneVertices = v;
% containerOneFN = containerOneMesh_h.FaceNormals;
hold on;

[partMesh, f, v] = Environment('Container2.ply', containerTwoLoc(1,4), containerTwoLoc(2,4), containerTwoLoc(3,4));
containerTwoMesh_h = partMesh;
containerTwoFaces = f;
containerTwoVertices = v;
% containerTwoFaceNorms = containerTwoMesh_h.FaceNormals;
hold on;

partMesh = Environment('Container3.ply', containerThreeLoc(1,4), containerThreeLoc(2,4), containerThreeLoc(3,4));
containerThreeMesh_h = partMesh;
containerThreeFaces = f;
containerThreeVertices = v;
% containerThreeFaceNorms = containerThreeMesh_h.FaceNormals;
hold on;
%%
%Safety Features - i.e eStop, encasing, 
eStopLoc = transl(1.7, 0.8, 0.2)*trotz(pi/2);             %Location needs to be fixed up, this is a random number5
barrierLoc = transl(0.6, 1, 0.2);

%eStop
partMesh = Environment('eStop.ply', eStopLoc(1,4), eStopLoc(2,4), eStopLoc(3,4)); %Rescale Estop lol
eStopMesh_h = partMesh;
hold on;
%%
%Barrier

partMesh = Environment('SafetyBarrier.ply', barrierLoc(1,4), barrierLoc(2,4), barrierLoc(3,4));
barrierMesh_h = partMesh;
hold on;

%Lights
% xOffset = ;
% yOffset = ;
% zOffset = ;
% partMesh = Environment('Lights.ply', xOffset, yOffset, zOffset);
% estopMesh_h = partMesh;
% hold on;


% Import Cutlery

spoonLoc = transl(1.2, 1, 0.25);
forkLoc = transl(1.3, 1, 0.25);
knifeLoc = transl(1.4, 1, 0.25);

[spoonMesh, spoonVertexCount, spoonVerts] = PlotCutlery('Spoon.ply', spoonLoc(1,4), spoonLoc(2,4), spoonLoc(3,4));
spoonMesh_h = spoonMesh; 

[forkMesh, forkVertexCount, forkVerts] = PlotCutlery('Fork.ply', forkLoc(1,4), forkLoc(2,4), forkLoc(3,4));
forkMesh_h = forkMesh;

[knifeMesh, knifeVertexCount, knifeVerts] = PlotCutlery('Knife.ply', knifeLoc(1,4), knifeLoc(2,4), knifeLoc(3,4));
knifeMesh_h = knifeMesh;



%% Collision avoidance?
%if collision == 1, then stop robot, if =0; continue movement
% collision checking should be done within movement section???? qMatrix
% depends on the movement of robot
qMatrix = deg2rad([-0.1745,5.4629,0.3316,-0.1745,2.4592,-0.1175]);           
% centerpnt = [1.1,1,0.25];
% side = 0.15;
% plotOptions.plotFaces = true;
% [Vertex,Faces,FaceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
 
collisionStatus = CheckForCollision(robot, qMatrix, benchMesh_h.Vertices, benchMesh_h.Faces, benchMesh_h.FaceNormals)
if collisionStatus == 1
  display('Collision Detected!!! Robot has paused');
  while collisionStatus == 1
  pause(1);
  end
  
else
    display('No collisions: Cutlery Magician safely moving~');
    return
end

% Check workspace area
% Want to see workspace area to see if all the items are within the arms
% reach 
% 
% stepRads = deg2rad(3);
% qlim = robot.model.qlim;
% 
% pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads +1))
% pointCloud = zeros(pointCloudeSize,3);
% counter = 1;
% 
% message = msgbox('Calculating Jaco workspace area');
% 
% for q1 = qlim(1,1):stepRads:qlim(1,2)
%     for q2 = qlim(2,1):stepRads:qlim(2,2)
%         for q3 = qlim(3,1):stepRads:qlim(3,2)
%             for q4 = qlim(4,1):stepRads:qlim(4,2)
%                 for q5 = qlim(5,1):stepRads:qlim(5,2)
%                     for q6 = 0
%                     q = [q1,q2,q3,q4,q5,q6];
%                     tr = robot.model.fkine(q);
%                     pointCloud(counter,:) = tr(1:3,4)';
%                     counter = counter + 1;
%                     end
%                 end
%             end
%         end
%     end
% end
% 
% [maxReach, workspaceVol] = convhull(pointCloud);
% 
% hold on 
% 
% maxReach = plot3(pointCloud(maxReach,1),pointCloud(maxReach,2),pointCloud(maxReach,3),'r.');
% 
% 
% msg = msgbox('Calculations complete, code paused');
% 
% pause();
% delete(maxReach);
%% Sensor data?

%% Estop Check
%This should be put within the movement function later. ie Check for estop
%and collisions before robot arm moves
% 
% if eStopButton ~= 0
%     display('EMERGENCY STOP');
%     while eStopPressed ~= 0
%         pause(1);
%     end
%     
% end

%% Movement
%Testing movement of arm from cutlery to containers. Will be changed to
%switch cases when sensor is added 
%EE stands for end effector 

steps = 50;
s = lspb(0,1,steps); %Scalar function
qMatrix = nan(steps, 6); %Memory allocation

%Use this bit of code to animate movement. Replace q as required
% for i = 1:steps
%     qMatrix(i,:) = (1-s(i))*q + s(i)*q;
%     robot.model.animate(qMatrix(i,:));
% end

%Pick up Spoon - HomePose to SpoonLoc, Spoon Loc to containerOne
%
qSpoon = robot.model.ikcon(spoonLoc);
qContainerOne = robot.model.ikcon(containerOneLoc);
qContainerTwo = robot.model.ikcon(containerTwoLoc);
qContainerThree = robot.model.ikcon(containerThreeLoc);
    
jacoMove(qHomePose, qSpoon, robot);
objectMove(qSpoon, qContainerThree, robot, spoonVerts, spoonVertexCount, spoonMesh_h);
jacoMove(qContainerThree, qHomePose, robot);
%

for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qHomePose + s(i)*qSpoon;
    robot.model.animate(qMatrix(i,:));
end


for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qSpoon + s(i)*qContainerThree;
    robot.model.animate(qMatrix(i,:));
    
    %Spoon model being brought to container    
    spoonEE = robot.model.fkine(qMatrix(i,:))*transl(0, 0, 0.05);
    spoonUpdatedPoints = [spoonEE*[spoonVerts,ones(spoonVertexCount,1)]']';
    spoonMesh_h.Vertices = spoonUpdatedPoints(:,1:3);
end

%Return to Home

for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qContainerThree + s(i)*qHomePose;
    robot.model.animate(qMatrix(i,:));
end


%%
qForcedCollision = transl(0.7, 1, 0.2);
            
            objectMove(qHomePose, qForcedCollision, robot, spoonVerts, spoonVertexCount, spoonMesh_h);
            collisionStatus = CheckForCollision(robot, qMatrix, barrierMesh_h.Vertices, barrierMesh_h.Faces, barrierMesh_h.FaceNormals)
                if collisionStatus == 1
                  
                  while collisionStatus == 1
                     
                     pause();
                  
                  
                  end
                  
                else
                    
                    jacoMove(qForcedCollision, app.qHomePose, app.robot);
                    return
                end