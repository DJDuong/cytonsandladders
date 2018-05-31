function BigMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
    if str2num(handles.ManualDie.String) > 2
        Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
    end
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
end