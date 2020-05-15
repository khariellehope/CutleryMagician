clf;

set(0,'DefaultFigureWindowStyle','docked');

workspace = [-4 4 -4 4 -4 4];
scale = 0.5;
jacoBase = transl(-0.2, 0.4,0.2);
%homePose = ;

%Get robot arm 

Jaco.GetJacoRobot();
Jaco.model.base = jacoBase;
Jaco.model.plot();




%%
% Import Environment

%Kitchen Bench

[kitf, kitv, kitdata] = plyread('kitchen.ply','tri');
kitVertexCount = size(kitv,1);
midPoint = sum(kitv)/kitVertexCount;
kitVerts = kitv - repmat(midPoint, kitVertexCount, 1);
kitpose = eye(4);
zOffset = 0;
xOffset = 0;
yOffset = 0;

vertexColours = [kitdata.vertex.red, kitdata.vertex.green, kitdata.vertex.blue]/255;
kitMesh_h = trisurf(kitf, kitVerts(:,1) + xOffset, kitVerts(:,2) + yOffset, kitVerts(:,3) + zOffset ...
    , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
hold on;
%Floor

floor = imread('floor.jpeg');
x = [2 -2.5; 2 -2.5];
y = [2 2; -2.5 -2.5];
z = [-0.6 -0.6; -0.6 -0.6];
floor_h = surf(x,y, z, 'CData', floor,'FaceColor','texturemap');

%Window Wall

window = imread('windowWall.jpg'); 
x = [1.1 -2.5; 1.1 -2.5];
y = [2 2; 2 2];
z = [1 1; -0.6 -0.6]; 
window_h = surf(x,y,z,'CData',window,'FaceCOlor','texturemap');
