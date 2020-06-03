function qNew = UpdateCartesian(robot, input, variable)
    switch input
        case 'x'
        qCurrent = robot.model.getpos();
        EEtransform = robot.model.fkine(qCurrent);
        EEtransform(1,4) = variable;
        qNew = robot.model.ikcon(EEtransform);  
        robot.model.animate(qNew);
        
        case 'y'
        qCurrent = robot.model.getpos();
        EEtransform = robot.model.fkine(qCurrent);
        EEtransform(2,4) = variable;
        qNew = robot.model.ikcon(EEtransform);  
        robot.model.animate(qNew);
        
        case 'z'
        qCurrent = robot.model.getpos();
        EEtransform = robot.model.fkine(qCurrent);
        EEtransform(3,4) = variable;
        qNew = robot.model.ikcon(EEtransform);  
        robot.model.animate(qNew);
        
    end 
end