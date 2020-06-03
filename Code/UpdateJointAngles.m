 %% GUI Functions
function qGUI = UpdateJointAngles(robot, joint,angle)
    qGUI = robot.model.getpos()  
    qGUI(1,joint) = deg2rad(angle)
    robot.model.animate(qGUI);

end