containerOneLoc = transl(0.9, 1, 0.2);
for z = 0.7:0.05:1.1
    line('XData', [0.8 2.5], 'YData', [1.2 1.2], 'Zdata', [z z], 'Color', [1 0 0]);
end

                startPoints = [];
                for x = 0.8:0.1:1.4
                    for y = 1.2
                        for z = 0.7:0.05:0.9
                            startPoints = [startPoints; x y z];
                        end
                        
                    end
                    
                end
                
                finishPoints = [];
                for x = 1.4:0.1:2.5
                    for y = 1.2
                        for z = 0.9:0.05:1.1
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
