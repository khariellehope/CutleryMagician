
function partMesh = Environment(object, xOffset, yOffset, zOffset)
    [f,v,data] = plyread(object,'tri');

    % Scale the colours to be 0-to-1 (they are originally 0-to-255
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

    % Plots the trisurf with the offset for each vertex
    partMesh = trisurf(f,v(:,1) + xOffset,v(:,2) + yOffset, v(:,3) + zOffset ...
      ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
   
end