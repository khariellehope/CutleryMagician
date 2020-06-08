containerOneLoc = transl(0.9, 1, 0.2);
for z = 0.2:0.05:0.5
    line('XData', [0.6 1.8], 'YData', [0.7 0.7], 'Zdata', [z z], 'Color', [0 1 0]);
    
end


    startPoints = [];
    for x = 0.6:0.1:1.2
        for y = 0.7
            for z = 0.2:0.05:0.35
                startPoints = [startPoints; x y z];
            end
            
        end
        
    end
    
    finishPoints = [];
    for x = 1.2:0.1:1.8
        for y = 0.7
            for z = 0.35:0.05:0.5
                finishPoints = [finishPoints; x y z];
            end
            
        end
        
    end



qContainerOne = robot.model.ikcon(containerOneLoc);
qStart = robot.model.getpos();
[statusFlag,handMesh_h] = jacoMoveTest(qStart,qContainerOne,robot, startPoints, finishPoints);
if statusFlag == 0;
     display('All is OK');
end
if statusFlag == 1;
     display('robot paused');
end
