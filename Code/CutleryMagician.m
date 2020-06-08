clc
clf
clear all

set(0,'DefaultFigureWindowStyle','docked');

% Model Jaco arm

jacoBase = transl(1.5, 1,0.25)*trotz(pi);
qHomePose = [0 0 0 0 0 0];                  %Change joint angles accordingly
qTest = deg2rad([-0.1466,5.4629,3.6849,0,3.1416,0]); 
%Get robot arm 
robot = Jaco;                            %Calling the Jaco class
robot.GetJacoRobot();                     %Calling the 'GetJacoRobot' function from class - This function creates the model using dh parameters
robot.PlotAndColourRobot();               %Use ply files to model realistic Jaco Robot
robot.model.base = jacoBase;              %Set base position
robot.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist'};
robot.model.plot(qTest, 'scale', robot.scale, 'workspace', robot.workspace);      %Plot model         
hold on;

%% Import Environment

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
x = [2 -2.; 2 -2];
y = [1.4 1.4; 1.4 1.4];
z = [1 1; -0.6 -0.6]; 
window_h = background('windowWall.jpg', x, y, z);


%%
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
eStopLoc = transl(0.6, -0.7,0.2);            %Location needs to be fixed up, this is a random number5
barrier1Loc = transl(0.6, 1, 0.2);
barrier2Loc = transl(1.8, 1, 0.2);
barrier3Loc = transl(0.9, 1.4, 0.2);
barrier4Loc = transl(1.4, 1.4, 0.2);


%eStop
partMesh = Environment('eStop.ply', eStopLoc(1,4), eStopLoc(2,4), eStopLoc(3,4)); %Rescale Estop lol
eStopMesh_h = partMesh;
hold on;

%Barrier

partMesh = Environment('SafetyBarrier.ply', barrier1Loc(1,4), barrier1Loc(2,4), barrier1Loc(3,4));
barrierMesh_h = partMesh;
hold on;

partMesh = Environment('SafetyBarrier.ply', barrier2Loc(1,4), barrier2Loc(2,4), barrier2Loc(3,4));
barrier2Mesh_h = partMesh;
hold on;

%Barriers against wall

partMesh = Environment('SafetyBarrier2.ply', barrier3Loc(1,4), barrier3Loc(2,4), barrier3Loc(3,4));
barrier3Mesh_h = partMesh;
hold on;

partMesh = Environment('SafetyBarrier2.ply', barrier4Loc(1,4), barrier4Loc(2,4), barrier4Loc(3,4));
barrier4Mesh_h = partMesh;
hold on;

%Light Curtain

for z = 0.2:0.05:0.6
    line('XData', [0.6 1.8], 'YData', [0.7 0.7], 'Zdata', [z z], 'Color', [1 0 0]);
end

%Warning Sign

x = [0.6 0.6; 0.6 0.6];
y = [0.9 1.1; 0.9 1.1];
z = [0.9 0.9; 0.7 0.7]; 
warningSign_h = background('WarningSign.jpg', x, y, z);

%% Simulate hand passing through light curtain


%hand
% 
% for y = 0.3:0.05:0.7
%     handLoc = transl(1, y, 0.4);
%     partMesh = Environment('hand2.ply', handLoc(1,4), handLoc(2,4), handLoc(3,4));
%     handMesh_h = partMesh;
%     drawnow;
%     delete(handMesh_h);
% end
% 
% handLoc = transl(1, 0.7, 0.4);
% partMesh = Environment('hand2.ply', handLoc(1,4), handLoc(2,4), handLoc(3,4));
% handMesh_h = partMesh;
%     
% pause();
%     
% delete(handMesh_h);


%% Import Cutlery

spoonLoc = transl(1.2, 1, 0.25);
forkLoc = transl(1.3, 1, 0.25);
knifeLoc = transl(1.4, 1, 0.25);

[spoonMesh, spoonVertexCount, spoonVerts] = PlotCutlery('Spoon.ply', spoonLoc(1,4), spoonLoc(2,4), spoonLoc(3,4));
spoonMesh_h = spoonMesh; 

[forkMesh, forkVertexCount, forkVerts] = PlotCutlery('Fork.ply', forkLoc(1,4), forkLoc(2,4), forkLoc(3,4));
forkMesh_h = forkMesh;

[knifeMesh, knifeVertexCount, knifeVerts] = PlotCutlery('Knife.ply', knifeLoc(1,4), knifeLoc(2,4), knifeLoc(3,4));
knifeMesh_h = knifeMesh;

%% Visual Servoing
%The Visual Servoing Section was taken from Lab 8 Solution 

pWarning = [212 512 812; 700 250 700]; %Image target points in image plane
targetPoints = [0.6 0.6 0.6;
    0.9 1.01 1.1; 
    0.71 0.9 0.71;];
qInitial = [-0.0026 1.5080 1.0060 1.6336 3.0708 0];
cam = CentralCamera('focal', 0.08', 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [500 500], 'name', 'CM Cam');
fps = 25;
lambda = 0.6 %Gain
depth = mean (targetPoints(1,:));
% Initialise visual servo sim 3d display
jcTr = robot.model.fkine(qInitial);
robot.model.animate(qInitial);
drawnow

cam.T = jcTr; %Plots cam 
cam.plot_camera('Tcam', jcTr, 'label', 'scale', 0.15);%Display points in cam
plot_sphere(targetPoints, 0.01, 'y');%Display points in 3d

% Initialise sim image view display
p = cam.plot(targetPoints, 'Tcam', jcTr); %project points to image

cam.clf();
cam.plot((pWarning), '*');
cam.hold(true);
cam.plot(targetPoints, 'Tcam', jcTr, 'o');%Create camera view
pause(2);
cam.hold(true);
cam.plot(targetPoints); %Plot initial View

vel_p = [];
uv_p = [];
history = [];
%%
%Visual Servo Loop 

ksteps = 0;
while true
    ksteps=ksteps +1;
    uv = cam.plot(targetPoints) %compute view of cam
    e = pWarning - uv; %feature error
    e = e(:);
    Zest = [];
    %compute jacobian
    if isempty(depth)
        %exact depth from sim
        pt = homtrans(inv(Tcam), pWarning);
        J = cam.visjac_p(uv, pt(3,:));
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    else 
        J = cam.visjac_p(uv, depth);
    end
     %compute velocity of cam in cam frame
    try
        v = lambda*pinv(J) * e;
    catch
        status = -1;
        return
    end
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    %compute robot jacobian and inverse
    J2 = robot.model.jacobn(qInitial);
    Jinv = pinv(J2);
    qp = Jinv*v; %get joint velocities
    %max angular velocity cannot exceed 180 degs
    ind = find(qp > pi);
    if ~isempty(ind)
        qp(ind) = pi;
    end
    ind = find(qp <= pi);
    if ~isempty(ind)
        qp(ind)=-pi;
    end
    
        %update joints
        q = qInitial + (1/fps) * qp;
        robot.model.animate(q);
        %get cam location
        Tc = robot.model.fkine(q');
        cam.T = Tc;
        drawnow
        %update history variables
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;
        history = [history hist];
        pause(1/fps);
        
        if ~isempty(200)&& (ksteps > 200)
            break;
        end
        
        qInitial = q;%update current joint position
end

Tr = robot.model.fkine(robot.model.getpos());

% Plot results (from Lab8Solution)
% figure()            
% plot_p(history,pWarning,cam)
% figure()
% plot_camera(history)
% figure()
% plot_vel(history)
% figure()
% plot_robjointpos(history)
% figure()
% plot_robjointvel(history)

%% Resolved motion rate control
% From Lab 9 Solution Question 1

t = 20;
deltaT = 0.02;
steps = t/deltaT;
delta = 2*pi/steps;
epsilon = 0.1; %Threshold val for manipulability/DLS
W = diag([1 1 1 0.1 0.1 0.1]);

m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,6);       % Array for joint anglesR
qdot = zeros(steps,6);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

%Set up trajectory, initial pose

s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*0.35 + s(i)*0.35; % Points in x
    x(2,i) = (1-s(i))*-0.55 + s(i)*0.55; % Points in y
    x(3,i) = 0.5 + 0.2*sin(i*delta); % Points in z
    theta(1,i) = 0;                 % Roll angle 
    theta(2,i) = 5*pi/9;            % Pitch angle
    theta(3,i) = 0;                 % Yaw angle
end

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