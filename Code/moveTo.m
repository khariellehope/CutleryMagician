% Movement
%Testing movement of arm from cutlery to containers. Will be changed to
%switch cases when sensor is added 
%EE stands for end effector 
classdef moveArm < handle
    properties
        spoonLoc = transl(1.2, 1, 0.25);
        forkLoc = transl(1.3, 1, 0.25);
        knifeLoc = transl(1.4, 1, 0.25);
    end
    
    methods
        function self = moveArm()
        end
        
        function moveToSpoon
            steps = 50;
            s = lspb(0,1,steps); %Scalar function
            qMatrix = nan(steps, 6); %Memory allocation
        end
        



%Pick up Spoon - HomePose to SpoonLoc, Spoon Loc to containerOne

qSpoon = robot.model.ikcon(spoonLoc);
qContainerOne = robot.model.ikcon(containerOneLoc);
qContainerTwo = robot.model.ikcon(containerTwoLoc);
qContainerThree = robot.model.ikcon(containerThreeLoc);


for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qHomePose + s(i)*qSpoon;
    robot.model.animate(qMatrix(i,:));
end


for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qSpoon + s(i)*qContainerThree;
    robot.model.animate(qMatrix(i,:));
    
    %Spoon model being brought to container    
    spoonEE = robot.model.fkine(qMatrix(i,:))*transl(0, 0, 0.05);
    spoonUpdatedPoints = [spoonEE*[spoonVerts,ones(spoonVertexCount,1)]']';
    spoonMesh_h.Vertices = spoonUpdatedPoints(:,1:3);
end

%Return to Home

for i = 1:steps
    qMatrix(i,:) = (1-s(i))*qContainerThree + s(i)*qHomePose;
    robot.model.animate(qMatrix(i,:));
end

    end
end


