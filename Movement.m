%Accepts robot object, whether the real robot will be controlled,
%trapezoidal trajectory parameters s and steps, the joint state to be
%reached, the scale of the simulated model, the game piece vertices needed
%to simulate movement of the game pieces, and handles to the edit text
%boxes of the GUI.
function Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
figure(1);
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
% if collision will happen on this path remake trajectory using pitstop as
% qNext and then plot from pitstop to cellMatrix.
%% Plot the trajectory in simulation
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
%           set(handles.IRStatus,'String', 'IR Intrusion Status: Detected and Waiting', 'BackgroundColor','r');
%           set(handles.RobotStatus, 'String','Robot Status: Stationary', 'BackgroundColor', 'w');
%         end
        set(handles.RobotStatus, 'String','Robot Status: Moving', 'BackgroundColor', 'g');
        set(handles.IRStatus,'String', 'IR Intrusion Status: No Intrusion Detected', 'BackgroundColor','w');
        robot.model.plot(qMatrix(i,:),'noarrow','workspace',robot.workspace, 'scale', scale);
        %setting joint angles to the gui
        handles.Joint1Box.String = qMatrix(i,1);
        handles.Joint2Box.String = qMatrix(i,2);
        handles.Joint3Box.String = qMatrix(i,3);
        handles.Joint4Box.String = qMatrix(i,4);
        handles.Joint5Box.String = qMatrix(i,5);
        handles.Joint6Box.String = qMatrix(i,6);
        handles.Joint7Box.String = qMatrix(i,7);
        %setting cartesian values to the gui
        endEffectorTr = robot.model.fkine(qMatrix(i,:));
        xInput = endEffectorTr(1,4);
        yInput = endEffectorTr(2,4);
        zInput = endEffectorTr(3,4);
        rpyValues = tr2rpy(endEffectorTr);
        rollInput = rpyValues(1);
        pitchInput = rpyValues(2);
        yawInput = rpyValues(3);
        handles.XInput.String = xInput; %for edit box
        handles.YInput.String = yInput; %for edit box
        handles.ZInput.String = zInput; %for edit box
        handles.RollInput.String = rollInput; %for edit box
        handles.PitchInput.String = pitchInput; %for edit box
        handles.YawInput.String = yawInput; %for edit box
        % Move Game Piece when play button is pressed - dont move if no
        % game piece vertices are passed into the move function
        if isempty(gamePieceVerts) ~= 1
            if handles.Play.Value == 1 || handles.MoveByPosition.Value == 1
                updatedPoints = [endEffectorTr * [gamePieceVerts,ones(gamePieceVertexCount,1)]']';  
                gamePiece.Vertices = updatedPoints(:,1:3);
            end
        end
    end
    set(handles.RobotStatus, 'String','Robot Status: Stationary', 'BackgroundColor', 'w');
%% Plot the trajectory in real robot
    if realRobot == 1
        for i=1:steps
            %Safety Check 1 - Emergency Stop toggled by GUI button
            while handles.EmergencyStop.Value == 1
                drawnow;
            end
            %Safety Check 2 - IR beam toggled by Arduino values
            IRSensor = Arduino();
            while IRSensor == 1
                drawnow;
                IRSensor = Arduino();
              set(handles.IRStatus,'String', 'IR Intrusion Status: Detected and Waiting', 'BackgroundColor','r');
              set(handles.RobotStatus, 'String','Robot Status: Stationary', 'BackgroundColor', 'w');
            end
            set(handles.RobotStatus, 'String','Robot Status: Moving', 'BackgroundColor', 'g');
            set(handles.IRStatus,'String', 'IR Intrusion Status: No Intrusion Detected', 'BackgroundColor','w');
            cute_multi_joint_msg.Vel = 1;
            cute_multi_joint_msg.JointStates = qMatrix(i,:);
            cute_multi_joint_client.call(cute_multi_joint_msg);

            handles.Joint1Box.String = qMatrix(i,1);
            handles.Joint2Box.String = qMatrix(i,2);
            handles.Joint3Box.String = qMatrix(i,3);
            handles.Joint4Box.String = qMatrix(i,4);
            handles.Joint5Box.String = qMatrix(i,5);
            handles.Joint6Box.String = qMatrix(i,6);
            handles.Joint7Box.String = qMatrix(i,7);

            endEffectorTr = robot.model.fkine(qMatrix(i,:));
            xInput = endEffectorTr(1,4);
            yInput = endEffectorTr(2,4);
            zInput = endEffectorTr(3,4);
            rpyValues = tr2rpy(endEffectorTr);
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
    set(handles.RobotStatus, 'String','Robot Status: Stationary', 'BackgroundColor', 'w');
end