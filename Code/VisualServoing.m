function []= VisualServoing()

%From Lab 8 Solution

pWarning = [212 512 812; 700 250 700]; %Image target points in image plane
targetPoints = [0.6 0.6 0.6; 0.9 1.01 1.1; 0.71 0.9 0.71;];
qInitial = [-0.0026 1.5080 1.0060 1.6336 3.0708 0];
cam = CentralCamera('focal', 0.08', 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [500 500], 'name', 'CM Cam');
fps = 25;
lambda = 0.6 %Gain
depth = mean (pWarning(1,:));

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
        robot.model.animate(q');
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
        pause(1/fps);
        
        if ~isempty(200)&& (ksteps > 200)
            break;
        end
        
        qInitial = q;%update current joint position
end
% Plot results (from Lab8Solution)
        figure()            
plot_p(history,pWarning,cam)
figure()
plot_camera(history)
figure()
plot_vel(history)
figure()
plot_robjointpos(history)
figure()
plot_robjointvel(history)
end

%% Functions for plotting

 function plot_p(history,uv_star,camera)
            %VisualServo.plot_p Plot feature trajectory
            %
            % VS.plot_p() plots the feature values versus time.
            %
            % See also VS.plot_vel, VS.plot_error, VS.plot_camera,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
            
            if isempty(history)
                return
            end
            figure();
            clf
            hold on
            % image plane trajectory
            uv = [history.uv]';
            % result is a vector with row per time step, each row is u1, v1, u2, v2 ...
            for i=1:numcols(uv)/2
                p = uv(:,i*2-1:i*2);    % get data for i'th point
                plot(p(:,1), p(:,2))
            end
            plot_poly( reshape(uv(1,:), 2, []), 'o--');
            uv(end,:)
            if ~isempty(uv_star)
                plot_poly(uv_star, '*:')
            else
                plot_poly( reshape(uv(end,:), 2, []), 'rd--');
            end
            axis([0 camera.npix(1) 0 camera.npix(2)]);
            set(gca, 'Ydir' , 'reverse');
            grid
            xlabel('u (pixels)');
            ylabel('v (pixels)');
            hold off
        end

       function plot_vel(history)
            %VisualServo.plot_vel Plot camera trajectory
            %
            % VS.plot_vel() plots the camera velocity versus time.
            %
            % See also VS.plot_p, VS.plot_error, VS.plot_camera,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
            if isempty(history)
                return
            end
            clf
            vel = [history.vel]';
            plot(vel(:,1:3), '-')
            hold on
            plot(vel(:,4:6), '--')
            hold off
            ylabel('Cartesian velocity')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z')
        end

        function plot_camera(history)
            %VisualServo.plot_camera Plot camera trajectory
            %
            % VS.plot_camera() plots the camera pose versus time.
            %
            % See also VS.plot_p, VS.plot_vel, VS.plot_error,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.

            if isempty(history)
                return
            end
            clf
            % Cartesian camera position vs time
            T = reshape([history.Tcam], 4, 4, []);
            subplot(211)
            plot(transl(T));
            ylabel('camera position')
            grid
            subplot(212)
            plot(tr2rpy(T))
            ylabel('camera orientation')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('R', 'P', 'Y');
            subplot(211)
            legend('X', 'Y', 'Z');
        end

        function plot_robjointvel(history)
          
            if isempty(history)
                return
            end
            clf
            vel = [history.qp]';
            plot(vel(:,1:6), '-')
            hold on
            ylabel('Joint velocity')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', '\omega_5', '\omega_6')
        end

 function plot_robjointpos(history)
          
            if isempty(history)
                return
            end
            clf
            pos = [history.q]';
            plot(pos(:,1:6), '-')
            hold on
            ylabel('Joint angle')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6')
        end
     
