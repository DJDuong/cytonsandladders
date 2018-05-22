% if boardValue == 29
%        trCurrent = robot.model.fkine(qCurrent);
%        cellXYZCurrent = trCurrent(1:3,4)';
%        cellRPYCurrent = tr2rpy(trCurrent);
%        startPos = cellXYZCurrent;
%        endPos = boardXYZMatrix(5,:)
%        startRPY = cellRPYCurrent;
%        endRPY = boardRPYMatrix(5,:)
%        qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
% end

switch boardValue
    case 3
       trCurrent = robot.model.fkine(qCurrent);
       cellXYZCurrent = trCurrent(1:3,4)';
       cellRPYCurrent = tr2rpy(trCurrent);
       startPos = cellXYZCurrent;
       endPos = boardXYZMatrix(21,:)
       startRPY = cellRPYCurrent;
       endRPY = boardRPYMatrix(21,:)
       qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
    case 14
       trCurrent = robot.model.fkine(qCurrent);
       cellXYZCurrent = trCurrent(1:3,4)';
       cellRPYCurrent = tr2rpy(trCurrent);
       startPos = cellXYZCurrent;
       endPos = boardXYZMatrix(27,:)
       startRPY = cellRPYCurrent;
       endRPY = boardRPYMatrix(27,:)
       qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
    case 18
       trCurrent = robot.model.fkine(qCurrent);
       cellXYZCurrent = trCurrent(1:3,4)';
       cellRPYCurrent = tr2rpy(trCurrent);
       startPos = cellXYZCurrent;
       endPos = boardXYZMatrix(20,:)
       startRPY = cellRPYCurrent;
       endRPY = boardRPYMatrix(20,:)
       qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
    case 12
       trCurrent = robot.model.fkine(qCurrent);
       cellXYZCurrent = trCurrent(1:3,4)';
       cellRPYCurrent = tr2rpy(trCurrent);
       startPos = cellXYZCurrent;
       endPos = boardXYZMatrix(2,:)
       startRPY = cellRPYCurrent;
       endRPY = boardRPYMatrix(2,:)
       qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
    case 26
       trCurrent = robot.model.fkine(qCurrent);
       cellXYZCurrent = trCurrent(1:3,4)';
       cellRPYCurrent = tr2rpy(trCurrent);
       startPos = cellXYZCurrent;
       endPos = boardXYZMatrix(13,:)
       startRPY = cellRPYCurrent;
       endRPY = boardRPYMatrix(13,:)
       qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
    case 29
       trCurrent = robot.model.fkine(qCurrent);
       cellXYZCurrent = trCurrent(1:3,4)';
       cellRPYCurrent = tr2rpy(trCurrent);
       startPos = cellXYZCurrent;
       endPos = boardXYZMatrix(5,:)
       startRPY = cellRPYCurrent;
       endRPY = boardRPYMatrix(5,:)
       qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
end
        