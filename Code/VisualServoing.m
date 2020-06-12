function []= VisualServoing()

%From Lab 8 Solution
pWarning = [310 510 610; 610 250 620]; %points to make triangle shape in camera view; asterisks are the desired points,
targetPoints = [0.8 0.8 0.8;  %points in the actual environment;
    1.35 1.5 1.65; 
    1.1 01.5 1.1];
%qInitial = [0 pi deg2rad(341) deg2rad(200) -pi/2 pi];
qInitial = [0 pi pi deg2rad(-238) deg2rad(133) deg2rad(150)];       %to have robot look at the warning sign/have it in view

cam = CentralCamera('focal', 0.08', 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [500 500], 'name', 'CM Cam');
fps = 100;
lambda = 0.6 %Gain
%depth = mean (targetPoints(1,:));
depth = 1;
% Initialise visual servo sim 3d display
jcTr = robot.model.fkine(qInitial); %Transform of EE initial pos
robot.model.animate(qInitial);
drawnow

cam.T = jcTr; %Plots cam 
cam.plot_camera('Tcam', jcTr, 'label', 'scale', 0.15);%Plot cam on EE
plot_sphere(targetPoints, 0.01, 'y');%Display points in 3d

% Initialise sim image view display
p = cam.plot(targetPoints, 'Tcam', jcTr); %project points to image (current view of cam)

%cam.clf();
cam.plot(pWarning, '*');%Plot desired view
cam.hold(true);
cam.plot(targetPoints, 'Tcam', jcTr, 'o');%What the camera Sees
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
    uv = cam.plot(targetPoints); %compute view of cam
    e = pWarning - uv; %feature error
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
end
