function MoveCyton()
%% Enabling robot movement and gripper
% Plug ethernet from modem to laptop.
% Plug ethernet from modem to upboard.
% Connect upboard powersource (socket to triangular prism ting to upboard)
% Set up Cyton on the table (screw onto). Connect powersource (socket to
% cyton).
% Connect usb from Cyton to Upboard.
% Open PuTTy, enter 192.168.0.253 and connect.
% Username and password is student.
% Turn on the Cyton and hold the arm up whilst entering- roslaunch cute_bringup cute_bringup.launch
% If red errors come up, ctrl c and keep retrying until the successful
% message appears with no red errors.
% Now open up another terminal in PuTTy by right clicking on the window bar and
% click duplicate session.
% Login with the same username and password again (student).
% Enter roslaunch robotics_cyton robotics-sim.launch.
% If there are errors, may need to turn off the cyton and retry from the
% first step.
% If successful and the green text saying You can start planning now! comes
% up, run the below setup section, then whichever other section.
%Setup Variables
rosshutdown
rosinit('192.168.0.102');
cute_enable_robot_client = rossvcclient('cute_enable_robot')
cute_enable_robot_msg = rosmessage(cute_enable_robot_client);

%To Enable/Disable the robot; Note: Hold onto the arm when disabling the robot.

cute_enable_robot_msg.EnableRobot = true; %false
cute_enable_robot_client.call(cute_enable_robot_msg);
%% Enabling Gripper
%Setup Variables

cute_enable_gripper_client = rossvcclient('claw_controller/torque_enable')
cute_enable_gripper_msg = rosmessage(cute_enable_gripper_client);

%To Enable/Disable the gripper

cute_enable_gripper_msg.TorqueEnable = true;% false
cute_enable_gripper_client.call(cute_enable_gripper_msg);
%% Open and Close the claw WILL USE THIS
% Setup Publisher and Message

cute_claw_publisher = rospublisher('/claw_controller/command');
cute_claw_msg = rosmessage(cute_claw_publisher);
% Publishing

cute_claw_msg.Data = -1.3; % Values must be between -1.5 (closed) and 0 (open)
% cute_claw_msg.Data = 0; % Values must be between -1.5 (closed) and 0 (open)
cute_claw_publisher.send(cute_claw_msg);

%% To create a trajectory from the current position of the arm to a transform WILL USE THIS.
% Creating Variables

cute_move_client = rossvcclient('/cute_move');
cute_move_msg = rosmessage(cute_move_client);
cute_execute_client = rossvcclient('/cute_execute_plan');
cute_execute_msg = rosmessage(cute_execute_client);
% Setting the positions (Example 0.1m, -0.15m, 0.2m)

cute_move_msg.Pose.Position.X = 0.1;
cute_move_msg.Pose.Position.Y = 0.1;
cute_move_msg.Pose.Position.Z = 0.2;
% Setting the orientation (Example 0 rads, PI rads, 0 rads)

quat = eul2quat(deg2rad([0,90,90]), 'XYZ');%Remember: Read the help file to understand how to use this function (’help eul2quat’)
cute_move_msg.Pose.Orientation.W = quat(1);
cute_move_msg.Pose.Orientation.X = quat(2);
cute_move_msg.Pose.Orientation.Y = quat(3);
cute_move_msg.Pose.Orientation.Z = quat(4);
% Sending the command to the planner

response = cute_move_client.call(cute_move_msg);
% At this point you should create a visualisation of the trajectory to ensure no collisions along the trajectory and correct final orientation before moving the robot. Retrieve the plan by:

cute_execute_msg.JointTrajectory = response.JointTrajectory;
% Send the trajectory to the robot

cute_execute_client.call(cute_execute_msg);

%% To move multiple joints to selected positions MAY USE THIS
% Setup Variables

cute_multi_joint_client = rossvcclient('/cute_multi_joint');
cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
% Setting values and sending to the robot
robot = Cyton;
startPos = [-0.05 0.1 0.3]
endPos = [0.05 0.1 0.3]
rpy = deg2rad([-90 0 0])
qMatrix = RMRS(robot.model, startPos, endPos, rpy);
robot.model.teach(qMatrix)
areal = 1; % acceleration, doesnt do anything...
vreal = 1; % velocity slowest to fastest = 0 to 1.

cute_multi_joint_msg.Vel = vreal;
cute_multi_joint_msg.Acc = areal;
[rows,columns] = size(qMatrix)
for i = 1:rows
    i 
    cute_multi_joint_msg.JointStates = qMatrix(i,:);
    cute_multi_joint_client.call(cute_multi_joint_msg);
end
%% Initial Start
% Setup Variables

cute_multi_joint_client = rossvcclient('/cute_multi_joint');
cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
areal = 1; % acceleration, doesnt do anything...
vreal = 1; % velocity slowest to fastest = 0 to 1.
cute_multi_joint_msg.Vel = vreal;
cute_multi_joint_msg.Acc = areal;
cute_multi_joint_msg.JointStates = [0 0 0 0 0 0 0];
cute_multi_joint_client.call(cute_multi_joint_msg);
%% To move a single joint to selected position
% Setup Variables

cute_single_joint_client = rossvcclient('/cute_single_joint');
cute_single_joint_msg = rosmessage(cute_single_joint_client);
% Setting values and sending to the robot

cute_single_joint_msg.JointNumber = 1;% Joints 0-6
cute_single_joint_msg.Angle = deg2rad([80]);% (Rads)
cute_single_joint_client.call(cute_single_joint_msg);
end