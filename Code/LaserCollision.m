function lightCurtainStatus = LaserCollision(startPoints, finishPoints, vertex, faces, faceNormals, returnOnceFound) %Checks for collisions with containers
%This part of the code was made using code from Lab 5 Solutions and the
%IsCollision.m file

if nargin < 6
    returnOnceFound = true;
end
lightCurtainStatus = 0;            %Set status flag as 0 first - Without this, the flag was always 

     % Go through each link and also each triangle face
    for i = 1 : size(startPoints,1)    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,startPoints(i,:),finishPoints(i,:)); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                %plot3(intersectP(1),intersectP(2),intersectP(3),'b*');
                display('Intersection');
                lightCurtainStatus = 1;
            if returnOnceFound
                    return
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


