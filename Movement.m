function Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%% Get the current joint state of real robot or simulation
if realRobot == 1
    cute_multi_joint_client = rossvcclient('/cute_multi_joint');
    cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
    stateSub = rossubscriber('/joint_states');
    receive(stateSub,2);
    msg = stateSub.LatestMessage;
    currentJointAngles = msg.Position;
    qCurrent = currentJointAngles(1:7)';
else
    qCurrent = robot.model.getpos();
end
%% Get the trajectory
    for i=1:steps
        % Use trapezoidal velocity interpolation to find the next joint
        % angle.
        qTry = (1-s(i))*qCurrent + s(i)*qNext;
        qMatrix(i,:) = qTry;
    end
%% Test trajectory for collision
% isCollision = CheckCollision(robot.model, qMatrix, vertex, faces, faceNormals)
%% Plot the trajectory in simulation
    for i=1:steps
        % Interrupt While loop that executes upon Emergency Stop toggle
        % button being pressed. the drawnow allows interruption, and once
        % the Emergency Stop toggle button is depressed, the movement
        % continues.
        while handles.EmergencyStop.Value == 1
        drawnow;
        end
%         IRSensor = Arduino()
%         while IRSensor == 1
%             drawnow;
%             IRSensor = Arduino()
%         end
        robot.model.plot(qMatrix(i,:),'noarrow','workspace',robot.workspace, 'scale', scale);
        joint1 = qMatrix(i,1);
        joint2 = qMatrix(i,2);
        joint3 = qMatrix(i,3);
        joint4 = qMatrix(i,4);
        joint5 = qMatrix(i,5);
        joint6 = qMatrix(i,6);
        joint7 = qMatrix(i,7);
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:));
        xInput = endeffector(1,4);
        yInput = endeffector(2,4);
        zInput = endeffector(3,4);
        rpyValues = tr2rpy(endeffector);
        rollInput = rpyValues(1);
        pitchInput = rpyValues(2);
        yawInput = rpyValues(3);
        handles.XInput.String = xInput; %for edit box
        handles.YInput.String = yInput; %for edit box
        handles.ZInput.String = zInput; %for edit box
        handles.RollInput.String = rollInput; %for edit box
        handles.PitchInput.String = pitchInput; %for edit box
        handles.YawInput.String = yawInput; %for edit box
        % Move Game Piece
        updatedPoints = [endEffector * [gamePieceVerts,ones(gamePieceVertexCount,1)]']';  
        gamePiece.Vertices = updatedPoints(:,1:3);
    end
%% Plot the trajectory in real robot
    if realRobot == 1
        for i=1:steps
        %Safety Check 1 - Emergency Stop toggled by GUI button
        while handles.EmergencyStop.Value == 1
            drawnow;
        end
        %Safety Check 2 - IR beam toggled by Arduino values
%         IRSensor = Arduino();
%         while IRSensor == 1
%             drawnow;
%             IRSensor = Arduino();
%         end
        cute_multi_joint_msg.JointStates = qMatrix(i,:);
        cute_multi_joint_client.call(cute_multi_joint_msg);
        joint1 = qMatrix(i,1);
        joint2 = qMatrix(i,2);
        joint3 = qMatrix(i,3);
        joint4 = qMatrix(i,4);
        joint5 = qMatrix(i,5);
        joint6 = qMatrix(i,6);
        joint7 = qMatrix(i,7);
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:));
        xInput = endeffector(1,4);
        yInput = endeffector(2,4);
        zInput = endeffector(3,4);
        rpyValues = tr2rpy(endeffector);
        rollInput = rpyValues(1);
        pitchInput = rpyValues(2);
        yawInput = rpyValues(3);
        handles.XInput.String = xInput; %for edit box
        handles.YInput.String = yInput; %for edit box
        handles.ZInput.String = zInput; %for edit box
        handles.RollInput.String = rollInput; %for edit box
        handles.PitchInput.String = pitchInput; %for edit box
        handles.YawInput.String = yawInput; %for edit box
        end
    end