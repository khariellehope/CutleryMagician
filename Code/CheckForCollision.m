function [collisionStatus, transform] = CheckForCollision(robot, qMatrix, vertex, faces, faceNormals, returnOnceFound) %Checks for collisions with Environment
%This part of the code was made using code from Lab 5 Solutions and the IsCollision.m file

if nargin < 6
    returnOnceFound = true;
end
collisionStatus = 0;            %Set status flag as 0 first - Without this, flag value output was always 0

for qIndex = 1:size(qMatrix,1)

    % Get the transform of every joint (i.e. start and end of every link)
    % on the robot to pass into LinePlaneIntersection
    % tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    transform = zeros(4,4,robot.model.n+1);
    transform(:,:,1) = robot.model.base;
    Links = robot.model.links;
    for i = 1 : robot.model.n
        transform(:,:,i+1) = transform(:,:,i) * trotz(qMatrix(i)+Links(i).offset) * transl(0,0,Links(i).d) * transl(Links(i).a,0,0) * trotx(Links(i).alpha);
    end

    % Go through each link and also each triangle face
    for i = 1 : size(transform,3)-1             %Goes through the transforms of the joints
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,transform(1:3,4,i)',transform(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:));
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                collisionStatus = 1;
            if returnOnceFound
                    return
                end                 
            end
        end    
    end
end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end


