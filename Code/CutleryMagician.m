%-----------------------------Reference List -------------------%
% Ply Model Files:
%       Simulated Hand OBJ files:
%       https://free3d.com/3d-model/freerealsichand-85561.html (Free3D)
%       
%       Kitchen Bench:
%       https://free3d.com/3d-model/kitchen-97041.html
%
%       Cutlery (Spoons, Forks, Knives):
%       Spoon: https://free3d.com/3d-model/spoon-16202.html
%       Fork: https://free3d.com/3d-model/-fork-v2--169267.html
%       Knife: https://free3d.com/3d-model/bread-knife-v1--471486.html
%       
%       Emergency Stop
%       https://free3d.com/3d-model/emergency-stop-button-941195.html
%
%       Safety Sign
%       https://au.rs-online.com/web/p/warning-signs/8578719/
%
%
%           ---- Various Robotics Lab Solutions were used throughout the
%           code and are referenced at the top of functions ------
%       


clc
clf
clear all

set(0,'DefaultFigureWindowStyle','docked');

% Model Jaco arm

jacoBase = transl(2.3, 1.85, 0.7);
homeLoc = transl(2.3, 1.85, 0.7);
qHomePose = [pi/2 pi deg2rad(341) pi/2 2*pi 0];                  %Change joint angles accordingly
%qTest = deg2rad([-0.1466,5.4629,3.6849,0,3.1416,0]); 
%Get robot arm 
robot = Jaco;                            %Calling the Jaco class
robot.GetJacoRobot();                     %Calling the 'GetJacoRobot' function from class - This function creates the model using dh parameters
robot.PlotAndColourRobot();               %Use ply files to model realistic Jaco Robot
robot.model.base = jacoBase;              %Set base position
robot.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist'};
robot.model.plot(qHomePose, 'scale', robot.scale, 'workspace', robot.workspace);      %Plot model         
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
x = [- 6 -6; 6 6];
y = [-2 2; -2 2];
z = [-0.5 -0.5; -0.5 -0.5];
floor_h = background('floor.jpeg', x, y, z);

%Window Wall
x = [4 -4; 4 -4];
y = [2.2 2.2; 2.2 2.2];
z = [2 2; -0.5 -0.5]; 
window_h = background('windowWall.jpg', x, y, z);


%%
%
%Import containers
%Locations of each container (hard coded):
containerOneLoc = transl(1.6, 1.4, 0.6)*trotx(pi);
containerTwoLoc = transl(1.6, 1.6, 0.6)*trotx(pi);
containerThreeLoc = transl(1.6, 1.8, 0.6)*trotx(pi);


[partMesh, f, v] = Environment('Container1.ply', containerOneLoc(1,4), containerOneLoc(2,4), containerOneLoc(3,4));
containerOneMesh_h = partMesh;
containerOneFaces = f;
containerOneVertices = v;
hold on;

[partMesh, f, v] = Environment('Container2.ply', containerTwoLoc(1,4), containerTwoLoc(2,4), containerTwoLoc(3,4));
containerTwoMesh_h = partMesh;
containerTwoFaces = f;
containerTwoVertices = v;
hold on;

partMesh = Environment('Container3.ply', containerThreeLoc(1,4), containerThreeLoc(2,4), containerThreeLoc(3,4));
containerThreeMesh_h = partMesh;
containerThreeFaces = f;
containerThreeVertices = v;
hold on;

%%
%Safety Features - i.e eStop, encasing, 
eStopLoc = transl(2.6, 1.2, 0.6);            %Location needs to be fixed up, this is a random number5

%eStop
partMesh = Environment('eStop.ply', eStopLoc(1,4), eStopLoc(2,4), eStopLoc(3,4)); %Rescale Estop lol
eStopMesh_h = partMesh;
hold on;

%Barrier
wallLoc = transl(0.9, 2.1, 1.05);
partMesh = Environment('Wall.ply', wallLoc(1,4), wallLoc(2,4), wallLoc(3,4));
wallMesh_h = partMesh;
hold on;

%Light Curtain

for z = 0.7:0.05:1.1
    line('XData', [0.8 2.5], 'YData', [1.2 1.2], 'Zdata', [z z], 'Color', [1 0 0]);
end

%Warning Sign
x = [0.9 0.9; 0.9 0.9];
y = [1.4 1.6; 1.4 1.6];
z = [1.2 1.2; 1 1]; 

warningSign_h = background('WarningSign.jpg', x, y, z);


%% Import Cutlery

        spoonLoc = transl(1.65, 1.5, 0.6)*trotx(pi);
        spoonLocTwo = transl(1.6, 1.6, 0.6)*trotx(pi);
        spoonLocThree = transl(1.65, 1.3, 0.6)*trotx(pi);
        forkLoc = transl(1.75, 1.4, 0.6)*trotx(pi);
        forkLocTwo = transl(1.6, 1.65, 0.6)*trotx(pi)*trotz(pi/2);
        forkLocThree = transl(1.65, 1.7, 0.6)*trotx(pi);
        knifeLoc = transl(1.6, 1.25, 0.6)*trotx(pi);
        knifeLocTwo = transl(1.68, 1.35, 0.6)*trotx(pi);
        knifeLocThree = transl(1.71, 1.7, 0.6)*trotx(pi);
        containerOneLoc = transl(1.6, 1.4, 0.6)*trotx(pi);
        containerTwoLoc = transl(1.5, 1.6, 0.6)*trotx(pi);
        containerThreeLoc = transl(1.6, 1.8, 0.6)*trotx(pi);

[spoonMesh, spoonVertexCount, spoonVerts] = PlotCutlery('Spoon.ply', spoonLoc(1,4), spoonLoc(2,4), spoonLoc(3,4));
spoonMesh_h = spoonMesh; 

[spoonMesh, spoonVertexCountTwo, spoonVertsTwo] = PlotCutlery('Spoon.ply', spoonLocTwo(1,4), spoonLocTwo(2,4), spoonLocTwo(3,4));
spoonMeshTwo_h = spoonMesh; 

[spoonMesh, spoonVertexCountThree, spoonVertsThree] = PlotCutlery('Spoon.ply', spoonLocThree(1,4), spoonLocThree(2,4), spoonLocThree(3,4));
spoonMeshThree_h = spoonMesh; 

[forkMesh, forkVertexCount, forkVerts] = PlotCutlery('Fork.ply', forkLoc(1,4), forkLoc(2,4), forkLoc(3,4));
forkMesh_h = forkMesh;

[forkMesh, forkVertexCountTwo, forkVertsTwo] = PlotCutlery('Fork.ply', forkLocTwo(1,4), forkLocTwo(2,4), forkLocTwo(3,4));
forkMeshTwo_h = forkMesh;

[forkMesh, forkVertexCountThree, forkVertsThree] = PlotCutlery('Fork.ply', forkLocThree(1,4), forkLocThree(2,4), forkLocThree(3,4));
forkMeshThree_h = forkMesh;

[knifeMesh, knifeVertexCount, knifeVerts] = PlotCutlery('Knife.ply', knifeLoc(1,4), knifeLoc(2,4), knifeLoc(3,4));
knifeMesh_h = knifeMesh;

[knifeMesh, knifeVertexCountTwo, knifeVertsTwo] = PlotCutlery('Knife.ply', knifeLocTwo(1,4), knifeLocTwo(2,4), knifeLocTwo(3,4));
knifeMeshTwo_h = knifeMesh;

[knifeMesh, knifeVertexCountThree, knifeVertsThree] = PlotCutlery('Knife.ply', knifeLocThree(1,4), knifeLocThree(2,4), knifeLocThree(3,4));
knifeMeshThree_h = knifeMesh;

%% Movement
%Testing movement of arm from cutlery to containers. Will be changed to
%switch cases when sensor is added 
%EE stands for end effector 

qSpoon = robot.model.ikcon(spoonLoc);
qSpoonTwo = robot.model.ikcon(spoonLocTwo);
qSpoonThree = robot.model.ikcon(spoonLocThree);
qFork = robot.model.ikcon(forkLoc);
qForkTwo = robot.model.ikcon(forkLocTwo);
qForkThree = robot.model.ikcon(forkLocThree);
qKnife = robot.model.ikcon(knifeLoc);
qKnifeTwo = robot.model.ikcon(knifeLocTwo);
qKnifeThree = robot.model.ikcon(knifeLocThree);
qContainerOne = robot.model.ikcon(containerOneLoc);
qContainerTwo = robot.model.ikcon(containerTwoLoc*transl(0,0,-0.2));
qContainerThree = robot.model.ikcon(containerThreeLoc*transl(0,0,0.25));

homePoseTr = robot.model.fkine(qHomePose);
containerOneTr = robot.model.fkine(qContainerOne);
containerTwoTr = robot.model.fkine(qContainerTwo);
containerThreeTr = robot.model.fkine(qContainerThree);
spoonTr = robot.model.fkine(qSpoon);
spoonTwoTr = robot.model.fkine(qSpoonTwo);
spoonThreeTr = robot.model.fkine(qSpoonThree);
forkTr = robot.model.fkine(qFork);
forTwoTr = robot.model.fkine(qForkTwo);
forkThreeTr = robot.model.fkine(qForkThree);
knifeTr = robot.model.fkine(qKnife);
knifeTwoTr = robot.model.fkine(qKnifeTwo);
knifeThreeTr = robot.model.fkine(qKnifeThree);
      
%% 
%Sort spoons:
rmrc(homePoseTr, spoonLoc, robot);
objectMove(qSpoon, qContainerThree, robot, spoonVerts, spoonVertexCount, spoonMesh_h);
rmrc(containerThreeTr, spoonLocTwo, robot);
objectMove(qSpoonTwo, qContainerThree, robot, spoonVertsTwo, spoonVertexCountTwo, spoonMeshTwo_h);
rmrc(containerThreeTr, spoonLocThree, robot);
objectMove(qSpoonThree, qContainerThree, robot, spoonVertsThree, spoonVertexCountThree, spoonMeshThree_h);
rmrc(containerThreeTr, homeLoc, robot);


%%
%Sort forks:
rmrc(homePoseTr, forkLoc, robot);
objectMove(qFork, qContainerTwo, robot, forkVerts, forkVertexCount, forkMesh_h);
rmrc(containerTwoTr, forkLocTwo, robot);
objectMove(qForkTwo, qContainerTwo, robot, forkVertsTwo, forkVertexCountTwo, forkMeshTwo_h);
rmrc(containerTwoTr, forkLocThree, robot);
objectMove(qForkThree, qContainerTwo, robot, forkVertsThree, forkVertexCountThree, forkMeshThree_h);
rmrc(containerTwoTr, homeLoc, robot);

%%
%Sort Knives:
rmrc(homePoseTr, knifeLoc, robot);
objectMove(qKnife, qContainerOne, robot, knifeVerts, knifeVertexCount, knifeMesh_h);
rmrc(containerOneTr, knifeLocTwo, robot);
objectMove(qKnifeTwo, qContainerOne, robot, knifeVertsTwo, knifeVertexCountTwo, knifeMeshTwo_h);
rmrc(containerOneTr, knifeLocThree, robot);
robjectMove(qKnifeThree, qContainerOne, robot, knifeVertsThree, knifeVertexCountThree, knifeMeshThree_h);
rmrc(containerOneTr, homeLoc, robot);

%% Visual Servoing

%The Visual Servoing Section was taken from Lab 8 Solution 

%pWarning = [212 512 812; 700 250 700]; %Image target points in image plane
%pWarning = [320 700 720; 650 850 550];

pWarning = [310 510 610; 610 250 620]; %Coordinates for features of the warning sign in the image plane
targetPoints = [0.8 0.8 0.8; %Location of the sign features in the real world
    1.35 1.5 1.65; 
    1.1 01.5 1.1];

%qInitial = [0 pi deg2rad(341) deg2rad(200) -pi/2 pi];
qInitial = [    1.5708    3.1416    5.9516    1.1938    2.2619    1.5708];
%Starting pose of the robot arm 
cam = CentralCamera('focal', 0.08', 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [500 500], 'name', 'CM Cam'); %Intialise the camera
fps = 100; %How many frames the camera gets per second 
lambda = 0.6 %Gain

%depth = mean (targetPoints(1,:));
depth = 1;

% Initialise visual servo sim 3d display
jcTr = robot.model.fkine(qInitial); %Transform of EE initial pos
robot.model.animate(qInitial);
drawnow

cam.T = jcTr; %Plots cam 
cam.plot_camera('Tcam', jcTr, 'label', 'scale', 0.15);%Plot cam on end effector
plot_sphere(targetPoints, 0.01, 'y');%Display points in 3d

% Initialise sim image view display
p = cam.plot(targetPoints, 'Tcam', jcTr); %Plot the camera view of target points in cam view

%cam.clf();
cam.plot(pWarning, '*');%Plot desired view in image plane
cam.hold(true);
cam.plot(targetPoints, 'Tcam', jcTr, 'o');%What the camera Sees
pause(2);
% cam.hold(true);
% cam.plot(targetPoints); %Plot initial View

vel_p = [];
uv_p = [];
history = [];

%Visual Servo Loop 

ksteps = 0;
while true
    ksteps=ksteps +1;
    uv = cam.plot(targetPoints); %Plot current view of camera
    e = pWarning - uv; %Calculate error
    e = e(:);
    Zest = [];
    %compute jacobian
    if isempty(depth)
        %exact depth from sim
        pt = homtrans(inv(Tcam), targetPoints);
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
    ind = find(qp <- pi);
    if ~isempty(ind)
        qp(ind)=-pi;
    end
    
        %update joints
        q = qInitial + (1/fps) * qp';
        robot.model.animate(q);
        %get cam location
        Tc = robot.model.fkine(q);
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
        pause(1/fps)
        
        if ~isempty(200)&& (ksteps > 200)
            break;
        end
        
        qInitial = q;%update current joint position
end

%% Resolved motion rate control
% From Lab 9 Solution Question 1

T = robot.model.fkine(robot.model.getpos()); %Current transform
q0 = robot.model.getpos(); %Current joint angles

t = 20; %Total time of RMRC
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

homeT = robot.model.fkine(qHomePose);

s = lspb(0,1,steps);                % Trapezoidal trajectory scalar (Create scalar function)

%Speficy desired pose of endefffector for RMRC
for i=1:steps
    x(1,i) = (1-s(i))*T(1,4) + s(i)*homeT(1,4); % Points in x
    x(2,i) = (1-s(i))*T(2,4) + s(i)*homeT(2,4); % Points in y
    x(3,i) = (1-s(i))*T(3,4) + s(i)*homeT(3,4); % Points in z
    theta(1,i) = 0;                 % Roll angle 
    theta(2,i) = 0;            % Pitch angle
    theta(3,i) = 0;                 % Yaw angle
end

qMatrix(1,:) = robot.model.ikcon(T, q0);

for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

%figure(1)
%plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
robot.model.plot(qMatrix);



%% RMRC Testing
% This is from Lab 9 Solution

t = 1;
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

T1 = robot.model.fkine(robot.model.getpos());%Starting 
T2 = containerThreeLoc;%Desired
q = robot.model.getpos();

s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*T1(1,4) + s(i)*T2(1,4); % Points in x
    x(2,i) = (1-s(i))*T1(2,4) + s(i)*T2(2,4); % Points in y
    x(3,i) = (1-s(i))*T1(3,4) + s(i)*T2(3,4); % Points in z
    theta(1,i) = deg2rad(-180);                 % Roll angle 
    theta(2,i) = deg2rad(0);            % Pitch angle
    theta(3,i) = deg2rad(0);                 % Yaw angle
end

qMatrix(1,:) = robot.model.ikcon(T1, q);

for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;  % For plotting  
    robot.model.plot(qMatrix);
    
end

robot.model.plot(qMatrix);

%% Collision Detection

startPoints = [];
                for x = 0.8:0.1:1.4
                    for y = 1.2
                        for z = 0.7:0.05:0.9
                            startPoints = [startPoints; x y z];
                        end
                        
                    end
                    
                end
                
                finishPoints = [];
                for x = 1.4:0.1:2.5
                    for y = 1.2
                        for z = 0.9:0.05:1.1
                            finishPoints = [finishPoints; x y z];
                        end
                        
                    end
                    
                end  
                
        qContainerOne = robot.model.ikcon(containerOneLoc);
        display('Light Curtain Activated');
        qStart = robot.model.getpos();
        display('No collisions: Robot safely moving ~');
        [statusFlag, handMesh_h] = jacoMoveTest(qStart, qContainerOne, app.robot, startPoints, finishPoints);
        handMesh = handMesh_h;
        if statusFlag == 1;
          display('Collision Detected!!');
          display('Robot has paused: Press Reset');
        end
%% Test Light Curtain

qForcedCollision = robot.model.ikcon(forcedCollisionLoc);
            flag = jacoMoveToCollision(qHomePose, qForcedCollision, robot, barrierMesh_h);
            if flag == 1;
                display('Collision Detected!!');
                display('Robot has paused: Press Reset');
            end