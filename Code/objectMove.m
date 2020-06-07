function qMatrix = objectMove(qStart,qFinish,robot,Verts, VertexCount, mesh)
    
steps = 50;
s = lspb(0,1,steps); %Scalar function
qMatrix = nan(steps, 6); %Memory allocation

for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qStart + s(i)*qFinish;
    transform = robot.model.fkine(qMatrix(i,:));
    robot.model.animate(qMatrix(i,:));
    
    %Spoon model being brought to container    
    EE = robot.model.fkine(qMatrix(i,:))*transl(0, 0, 0.05);
    UpdatedPoints = [EE*[Verts,ones(VertexCount,1)]']';
    mesh.Vertices = UpdatedPoints(:,1:3);
end
  
end