function statusFlag = jacoMoveToCollision(qStart,qFinish,robot, mesh)

statusFlag = 0;
steps = 50;
s = lspb(0,1,steps); %Scalar function
qMatrix = nan(steps, 6); %Memory allocation

for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qStart + s(i)*qFinish;
    %transform = robot.model.fkine(qMatrix(i,:));
    qCurrent = robot.model.getpos();
    
    collisionStatus = CheckForCollision(robot, qCurrent, mesh.Vertices, mesh.Faces, mesh.FaceNormals); 
    if collisionStatus == 1
      
      statusFlag = 1;
      return
              
     end
    robot.model.animate(qMatrix(i,:)); 
 
end
  
end
