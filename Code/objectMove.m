function qMatrix = objectMove(qStart,qFinish,robot,Verts, VertexCount, mesh)
% This function is based off Lab __ Solutions
    
steps = 50;
s = lspb(0,1,steps); %Scalar function
qMatrix = nan(steps, 6); %Memory allocation

for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qStart + s(i)*qFinish;      %Using the Trapezoidal method for Trajectory Planning/Movement
    transform = robot.model.fkine(qMatrix(i,:));        
    robot.model.animate(qMatrix(i,:));
    
    % Cutlery model being brought to container    
    EE = robot.model.fkine(qMatrix(i,:));               %Get transform of end effector, when it goes through the joint angles to get to the end location    
    UpdatedPoints = [EE*[Verts,ones(VertexCount,1)]']'; % This part of the code is from the PuttingSimulatedObjectsIntoTheEnvironment.m code
    mesh.Vertices = UpdatedPoints(:,1:3);
end
  
end