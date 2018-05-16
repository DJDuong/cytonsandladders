function qMatrix = RMRS(cyton, startPos, endPos, rpy)
    t = 3;             % Total time (s)
    deltaT = 0.02;      % Control frequency
    steps = t/deltaT;   % No. of steps for simulation
    delta = 2*pi/steps; % Small angle change
    epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
    W = eye(6);         % Weighting matrix for the velocity vector

    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,7);       % Array for joint anglesR
    qdot = zeros(steps,7);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    positionError = zeros(3,steps); % For plotting trajectory error
    angleError = zeros(3,steps);    % For plotting trajectory error

    s = lspb(0,1,steps);             % Trapezoidal trajectory scalar
    for i=1:steps
        x(1,i) = (1-s(i))*startPos(1) + s(i)*endPos(1); % Points in x
        x(2,i) = (1-s(i))*startPos(2) + s(i)*endPos(2); % Points in y
        x(3,i) = (1-s(i))*startPos(3) + s(i)*endPos(3);  % Points in z
        theta(1,i) = rpy(1);                      % Roll angle 
        theta(2,i) = rpy(2);                      % Pitch angle
        theta(3,i) = rpy(3);                      % Yaw angle
    end
 
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = zeros(1,7);                                                            % Initial guess for joint angles
    qMatrix(1,:) = cyton.ikcon(T,q0);                                           % Solve joint angles to achieve first waypoint

    for i = 1:steps-1
        T = cyton.fkine(qMatrix(i,:));                                          % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = cyton.jacob0(qMatrix(i,:));                                         % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon                                                       % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
        for j = 1:7                                                             % Loop through joints 1 to 7
            if qMatrix(i,j) + deltaT*qdot(i,j) < cyton.qlim(j,1)                % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > cyton.qlim(j,2)            % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
    end
    plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
end