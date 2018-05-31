%Called to ensure that a large movement is split into two smaller trajectories
%rather than one large trajectory that could cause collision.
function CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, handles)
    if collisioncheck == 1
    % COLLISION CHECK - Test trajectory against all environment objects before running Movement.
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
        for i=1:steps
            % Use trapezoidal velocity interpolation to find the next joint
            % angle.
            qTry = (1-s(i))*qCurrent + s(i)*qNext;
            qMatrix(i,:) = qTry;
        end
        isCollision = CheckCollision(robot.model, qMatrix, vertex, faces, faceNormals)
        if isCollision == true
            Movement(robot, realRobot, s, steps, pitstop, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
        end
        Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
    else
    % FAKE COLLISION CHECK - Assume the need to pitstop if the Die Value is greater than 3.
        if str2num(handles.ManualDie.String) > 3
            Movement(robot, realRobot, s, steps, pitstop, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
        end
        Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
    end
end