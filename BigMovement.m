%Called to ensure that a large movement is split into two smaller trajectories
%rather than one large trajectory that could cause collision.
function BigMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
    if str2num(handles.ManualDie.String) > 3
        Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
    end
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
end