% This function will be used to plot the mesh of the cutlery, and calculate
%vertex count etc.
%This function was previously used in Assignment 1

function [partMesh, partVertexCount, partVerts] = PlotCutlery(object, xOffset, yOffset, zOffset)
    [f,v,data] = plyread(object,'tri');

        partVertexCount = size(v,1);                    % Get vertex count
        partMidPoint = sum(v)/partVertexCount;        % Move center point to origin
        partVerts = v - repmat(partMidPoint,partVertexCount,1);
        partPose = eye(4);                        % Create a transform to describe the location (at the origin, since it's centered

        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/ 255; % Scale the colours to be 0-to-1 (they are originally 0-to-255

        % plot the trisurf
        partMesh = trisurf(f,v(:,1)+ xOffset, v(:,2)+ yOffset, v(:,3)+zOffset...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
     

end
