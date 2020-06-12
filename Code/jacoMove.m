function qMatrix = jacoMove(qStart,qFinish,robot)
    
steps = 50;
s = lspb(0,1,steps); %Scalar function
qMatrix = nan(steps, 6); %Memory allocation

for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qStart + s(i)*qFinish;
    transform = robot.model.fkine(qMatrix(i,:));
   
    robot.model.animate(qMatrix(i,:));
    
     
    
    
    
end
  
end
