%% Motion Via Waypoint
function Waypoint(cyton, startXYZ, endXYZ, offset)
    startPoint = JointPos(cyton, startXYZ);
    endPoint = JointPos(cyton ,endXYZ);
    wayPoint = [nan nan offset];
    for i = 1 : 2
        wayPoint(i)=(startXYZ(i)+endXYZ(i))/2;
    end
    wayPoint = JointPos(cyton, wayPoint);
    cyton.plot(Interpolation(startPoint, wayPoint);
    cyton.plot(Interpolation(wayPoint, endPoint);
end
%% Generate Joint Position Matrix
function jointPos = JointPos(cyton, pointCoordinates)
    translation = transl(pointCoordinates); %produce transform from given xyz coords
    jointPos = cyton.ikcon(translation);    %return joint positions of transform
end
%% Arm Pathing Function
function jointMatrix = Interpolation(initialJoint, finalJoint)
    steps = 50; %steps robot takes to move from intitial position to final position
    scalar = lspb(0,1,steps);   %create the scalar function
    jointMatrix = nan(steps,7); %Create memory allocation for variables
    for i = 1:steps
        jointMatrix(i,:) = (1-scalar(i))*initialJoint + scalar(i)*finalJoint;   % Generate interpolated joint angles
    end
end