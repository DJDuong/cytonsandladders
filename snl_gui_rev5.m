function varargout = snl_gui_rev5(varargin)
% SNL_GUI MATLAB code for snl_gui.fig
%      SNL_GUI, by itself, creates a new SNL_GUI or raises the existing
%      singleton*.
%
%      H = SNL_GUI returns the handle to a new SNL_GUI or the handle to
%      the existing singleton*.
%
%      SNL_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SNL_GUI.M with the given input arguments.
%
%      SNL_GUI('Property','Value',...) creates a new SNL_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before snl_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to snl_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help snl_gui

% Last Modified by GUIDE v2.5 27-May-2018 17:36:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @snl_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @snl_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before snl_gui is made visible.
function snl_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to snl_gui (see VARARGIN)
% ====================BELOW IS RUN BEFORE GUI APPEARS=====================%

    %> global variables where some have default values; initialised before
    % the gui is made visible
    global robot;
    global eStop;
    global boardValue;
    global boardValueMin;
    global boardValueMax;
    global dieValueMin;
    global dieValueMax;
    global qZero;
    global qCurrent;
    global realRobot;
    global rosIP;
    %cells to be used in board algorithm.
    %tim, maybe look into creating an array that can map xyz values to the
    %cell locations. there is a map function on matlab.
    global pitstop;
    global joint1;
    global joint2;
    global joint3;
    global joint4;
    global joint5;
    global joint6;
    global joint7;
    global xInput;
    global yInput;
    global zInput;
    global rollInput;
    global pitchInput;
    global yawInput;
    global joint1Min;
    global joint1Max;
    global joint2Min;
    global joint2Max;
    global joint3Min;
    global joint3Max;
    global joint4Min;
    global joint4Max;
    global joint5Min;
    global joint5Max;
    global joint6Min;
    global joint6Max;
    global joint7Min;
    global joint7Max;
    global steps;
    global s;
    global cellMatrix;
    
    cellMatrix = xlsread('jointAngles.xlsx')
    pitstop = [-0.1534 0.8145 -0.0215 2.0678 -0.0215 -1.2855 1.3698]
    joint1Min = -150;
    joint1Max = 150;
    joint2Min = -105;
    joint2Max = 105;
    joint3Min = -150;
    joint3Max = 150;
    joint4Min = -105;
    joint4Max = 105;
    joint5Min = -105;
    joint5Max = 105;
    joint6Min = -105;
    joint6Max = 105;
    joint7Min = -150;
    joint7Max = 150;
    steps = 20;
    s = lspb(0,1,steps);
    qZero = [0 0 0 0 0 0 0];
    %starting with robot at cell1
    boardValue = 1;
    %realRobot = 1 for real robot usage.
    realRobot = 0;
    rosIP = '192.168.0.100';
    % Range of dieValue - six faces, 1-6.
    dieValueMin = 1;
    dieValueMax = 6;
    
    % Number of spaces on the board - there are 30 spaces.
    boardValueMin = 1;
    boardValueMax = 30;
    
    % initialise the eStop variable so that the toggle changes it to 1 upon
    % stopping.
    eStop = 0;
    
%     % realistic variable = 1 - realistic arms, 2 - simple shape arms, 3 - 
%     % stick model arms.
%     realistic = 3;          
    
    % handle for objects cyton class
    robot = Cyton();
    
    %Tims ply stuff - environment.
    [Tri,Pts] = plyread('environment/Cage5.ply','tri');                                    
   trisurf(Tri,Pts(:,1),Pts(:,2),Pts(:,3));
   hold on
   %board
   [Tri,Pts] = plyread('environment/Board2.ply','tri');                                  
   trisurf(Tri,Pts(:,1)+0.2075,Pts(:,2)+0.4,Pts(:,3)+0.02);
   %button
   [Tri,Pts] = plyread('environment/button.ply','tri');                                  
   trisurf(Tri,Pts(:,1)-0.2075,Pts(:,2)+0.2,Pts(:,3));
   camlight;
    % show cyton at pitstop angle state
    robot.model.plot(pitstop, 'scale', 0.5);
    robot.PlotAndColourRobot();
   
    if realRobot == 1
        % Enabling robot movement and gripper
        % Plug ethernet from modem to laptop.
        % Plug ethernet from modem to upboard.
        % Connect upboard powersource (socket to triangular prism ting to upboard)
        % Set up Cyton on the table (screw onto). Connect powersource (socket to
        % cyton).
        % Connect usb from Cyton to Upboard.
        % Open PuTTy, enter 192.168.0.??? and connect.
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
        rosshutdown;
        rosinit(rosIP);
        cute_enable_robot_client = rossvcclient('cute_enable_robot')
        cute_enable_robot_msg = rosmessage(cute_enable_robot_client);

        %To Enable/Disable the robot; Note: Hold onto the arm when disabling the robot.

        cute_enable_robot_msg.EnableRobot = true; %false
        cute_enable_robot_client.call(cute_enable_robot_msg);
        %Enabling Gripper
        %Setup Variables

        cute_enable_gripper_client = rossvcclient('claw_controller/torque_enable')
        cute_enable_gripper_msg = rosmessage(cute_enable_gripper_client);

        %To Enable/Disable the gripper

        cute_enable_gripper_msg.TorqueEnable = true;% false
        cute_enable_gripper_client.call(cute_enable_gripper_msg);
        
        %Moving the robot arm to pitstop state
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
        stateSub = rossubscriber('/joint_states');
        receive(stateSub,2)
        msg = stateSub.LatestMessage;
        cute_multi_joint_msg.JointStates = pitstop;
        cute_multi_joint_client.call(cute_multi_joint_msg);
        
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
        joint1 = qCurrent(1)
        joint2 = qCurrent(2)
        joint3 = qCurrent(3)
        joint4 = qCurrent(4)
        joint5 = qCurrent(5)
        joint6 = qCurrent(6)
        joint7 = qCurrent(7)
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box
        
        endeffector = robot.model.fkine(pitstop)
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
    else
    end
% ====================ABOVE IS RUN BEFORE GUI APPEARS=====================%
% Choose default command line output for snl_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes snl_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = snl_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;  

% --- Executes on button press in RandomDie.
function RandomDie_Callback(hObject, eventdata, handles)
% hObject    handle to RandomDie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dieValue
global dieValueMax
dieValue = randi(dieValueMax)
handles.ManualDie.String = dieValue; %for edit box

function ManualDie_Callback(hObject, eventdata, handles)
% hObject    handle to ManualDie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dieValue
global dieValueMin
global dieValueMax
    message = sprintf(['Input must be an integer, inclusive between ' ...
            num2str(dieValueMin) ' and ' num2str(dieValueMax) '.']);
    % getting the handle object, converting it to a double and setting it to a
    % temporary die value called manualDieValue.
    manualDieValue = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualDieValue)
        uiwait(warndlg(message));
        handles.ManualDie.String = 'Die Input';
        return;
    end
    % check if the die value entered is an integer outside of the dice
    % range.
    if (manualDieValue < dieValueMin || dieValueMax < manualDieValue) 
       uiwait(warndlg(message));
       handles.ManualDie.String = 'Die Input';
       return;
    end
    % check if the die value entered is an integer with decimal places
    % ('fix' rounds up/down the variable. This rounded value minus the 
    % original value will not equal 0 if the value entered had decimal  
    % places).
    if fix(manualDieValue) - manualDieValue ~= 0
       uiwait(warndlg(message));
       handles.ManualDie.String = 'Die Input';
       return;
    end
    % setting the global variable for die value to the entered die value.
    dieValue = manualDieValue


% Hints: get(hObject,'String') returns contents of ManualDie as text
%        str2double(get(hObject,'String')) returns contents of ManualDie as a double


% --- Executes during object creation, after setting all properties.
function ManualDie_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ManualDie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ManualPositionValue_Callback(hObject, eventdata, handles)
% hObject    handle to ManualPositionValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global boardValue
global boardValueMin
global boardValueMax

    message = sprintf(['Input must be an integer, inclusive between ' ...
            num2str(boardValueMin) ' and ' num2str(boardValueMax) '.']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualBoardValue = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualBoardValue)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = 'Position Input';
        return;
    end
    % check if the die value entered is an integer outside of the dice
    % range.
    if (manualBoardValue < boardValueMin || boardValueMax < manualBoardValue) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = 'Position Input';
       return;
    end
    % check if the die value entered is an integer with decimal places
    % ('fix' rounds up/down the variable. This rounded value minus the 
    % original value will not equal 0 if the value entered had decimal  
    % places).
    if fix(manualBoardValue) - manualBoardValue ~= 0
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = 'Position Input';
       return;
    end
    % setting the global variable for die value to the entered die value.
    boardValue = manualBoardValue
    
% Hints: get(hObject,'String') returns contents of ManualPositionValue as text
%        str2double(get(hObject,'String')) returns contents of ManualPositionValue as a double


% --- Executes during object creation, after setting all properties.
function ManualPositionValue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ManualPositionValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in ThrowDie.
function ThrowDie_Callback(hObject, eventdata, handles)
% hObject    handle to ThrowDie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global eStop;
global dieValue;
for i = 0:50
        pause(0.3)
        % Interrupt While loop that executes upon Emergency Stop toggle
        % button being pressed. the drawnow allows interruption, and once
        % the Emergency Stop toggle button is depressed, the movement
        % continues.
        while eStop == 1
            drawnow;
        end
        % This is where the trajectory operation should occur.
        moveValue = moveValue + 1
        display(['This is a test; try the Emergency Stop button.'...
            'Movement Value is' num2str(moveValue)]);
        drawnow;
        
        % This is where the sensing of the die value should occur.
        % dieValue = sensordata
    end

% --- Executes on button press in SenseDie.
function SenseDie_Callback(hObject, eventdata, handles)
% hObject    handle to SenseDie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
message = sprintf(['Nothing in this function yet.']);
uiwait(warndlg(message))

% --- Executes on button press in MoveByDie.
function MoveByDie_Callback(hObject, eventdata, handles)
% hObject    handle to MoveByDie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dieValue
global boardValue;
global robot;
global eStop;
global qCurrent;
global realRobot;
global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;
global joint7;
global xInput;
global yInput;
global zInput;
global rollInput;
global pitchInput;
global yawInput;
global cellMatrix;
global steps;
global s;
    boardValue;
    boardValue = dieValue + boardValue;
    handles.ManualPositionValue.String = boardValue
%get the current joint state of real robot or simulation
if realRobot ==1
    cute_multi_joint_client = rossvcclient('/cute_multi_joint');
    cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
    stateSub = rossubscriber('/joint_states');
    receive(stateSub,2)
    msg = stateSub.LatestMessage;
    currentJointAngles = msg.Position;
    qCurrent = currentJointAngles(1:7)'
else
    qCurrent = robot.model.getpos()
end
%get the trajectory
    for i=1:steps
        % Use trapezoidal velocity interpolation to find the next joint
        % angle.
        qTry = (1-s(i))*qCurrent + s(i)*cellMatrix(boardValue,:)
        qMatrix(i,:) = qTry;
    end
%plot the trajectory in simulation
    for i=1:steps
        % Interrupt While loop that executes upon Emergency Stop toggle
        % button being pressed. the drawnow allows interruption, and once
        % the Emergency Stop toggle button is depressed, the movement
        % continues.
        while eStop == 1
        drawnow;
        end
        robot.model.plot(qMatrix(i,:));
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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
%plot the trajectory in real robot
    if realRobot == 1
        for i=1:steps
        while eStop == 1
        drawnow;
        end
        cute_multi_joint_msg.JointStates = qMatrix(i,:);
        cute_multi_joint_client.call(cute_multi_joint_msg);
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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
    switch boardValue
        case 3
            boardValue = 21
            handles.ManualPositionValue.String = boardValue
        case 14
            boardValue = 27
            handles.ManualPositionValue.String = boardValue
        case 18
            boardValue = 20
            handles.ManualPositionValue.String = boardValue
        case 12
            boardValue = 2
            handles.ManualPositionValue.String = boardValue
        case 26
            boardValue = 13
            handles.ManualPositionValue.String = boardValue
        case 29 
        boardValue = 5
        handles.ManualPositionValue.String = boardValue
    end
    %get the current joint state of real robot or simulation
if realRobot ==1
    cute_multi_joint_client = rossvcclient('/cute_multi_joint');
    cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
    stateSub = rossubscriber('/joint_states');
    receive(stateSub,2)
    msg = stateSub.LatestMessage;
    currentJointAngles = msg.Position;
    qCurrent = currentJointAngles(1:7)';
else
    qCurrent = robot.model.getpos();
end
%get the trajectory
    for i=1:steps
        % Use trapezoidal velocity interpolation to find the next joint
        % angle.
        qTry = (1-s(i))*qCurrent + s(i)*cellMatrix(boardValue,:);
        qMatrix(i,:) = qTry;
    end
%plot the trajectory in simulation
    for i=1:steps
        % Interrupt While loop that executes upon Emergency Stop toggle
        % button being pressed. the drawnow allows interruption, and once
        % the Emergency Stop toggle button is depressed, the movement
        % continues.
        while eStop == 1
        drawnow;
        end
        robot.model.plot(qMatrix(i,:));
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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
%plot the trajectory in real robot
    if realRobot == 1
        for i=1:steps
        while eStop == 1
        drawnow;
        end
        cute_multi_joint_msg.JointStates = qMatrix(i,:);
        cute_multi_joint_client.call(cute_multi_joint_msg);
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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

    % --- Executes on button press in MoveByPosition.
function MoveByPosition_Callback(hObject, eventdata, handles)
% hObject    handle to MoveByPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global boardValue;
global robot;
global eStop;
global realRobot;
global qCurrent;
global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;
global joint7;
global xInput;
global yInput;
global zInput;
global rollInput;
global pitchInput;
global yawInput;
global cellMatrix;
global steps;
global s;
boardValue
%get the current joint state of real robot or simulation
if realRobot ==1
    cute_multi_joint_client = rossvcclient('/cute_multi_joint');
    cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
    stateSub = rossubscriber('/joint_states');
    receive(stateSub,2)
    msg = stateSub.LatestMessage;
    currentJointAngles = msg.Position;
    qCurrent = currentJointAngles(1:7)'
else
    qCurrent = robot.model.getpos()
end
%get the trajectory
    for i=1:steps
        % Use trapezoidal velocity interpolation to find the next joint
        % angle.
        qTry = (1-s(i))*qCurrent + s(i)*cellMatrix(boardValue,:)
        qMatrix(i,:) = qTry;
    end
%plot the trajectory in simulation
    for i=1:steps
        % Interrupt While loop that executes upon Emergency Stop toggle
        % button being pressed. the drawnow allows interruption, and once
        % the Emergency Stop toggle button is depressed, the movement
        % continues.
        while eStop == 1
        drawnow;
        end
        robot.model.plot(qMatrix(i,:));
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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
%plot the trajectory in real robot
    if realRobot == 1
        for i=1:steps
        while eStop == 1
        drawnow;
        end
        cute_multi_joint_msg.JointStates = qMatrix(i,:);
        cute_multi_joint_client.call(cute_multi_joint_msg);
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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
    switch boardValue
        case 3
            boardValue = 21
            handles.ManualPositionValue.String = boardValue
        case 14
            boardValue = 27
            handles.ManualPositionValue.String = boardValue
        case 18
            boardValue = 20
            handles.ManualPositionValue.String = boardValue
        case 12
            boardValue = 2
            handles.ManualPositionValue.String = boardValue
        case 26
            boardValue = 13
            handles.ManualPositionValue.String = boardValue
        case 29 
        boardValue = 5
        handles.ManualPositionValue.String = boardValue
    end
    %get the current joint state of real robot or simulation
if realRobot ==1
    cute_multi_joint_client = rossvcclient('/cute_multi_joint');
    cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
    stateSub = rossubscriber('/joint_states');
    receive(stateSub,2)
    msg = stateSub.LatestMessage;
    currentJointAngles = msg.Position;
    qCurrent = currentJointAngles(1:7)';
else
    qCurrent = robot.model.getpos();
end
%get the trajectory
    for i=1:steps
        % Use trapezoidal velocity interpolation to find the next joint
        % angle.
        qTry = (1-s(i))*qCurrent + s(i)*cellMatrix(boardValue,:);
        qMatrix(i,:) = qTry;
    end
%plot the trajectory in simulation
    for i=1:steps
        % Interrupt While loop that executes upon Emergency Stop toggle
        % button being pressed. the drawnow allows interruption, and once
        % the Emergency Stop toggle button is depressed, the movement
        % continues.
        while eStop == 1
        drawnow;
        end
        robot.model.plot(qMatrix(i,:));
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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
%plot the trajectory in real robot
    if realRobot == 1
        for i=1:steps
        while eStop == 1
        drawnow;
        end
        cute_multi_joint_msg.JointStates = qMatrix(i,:);
        cute_multi_joint_client.call(cute_multi_joint_msg);
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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

% --- Executes on button press in Reset.
function Reset_Callback(hObject, eventdata, handles)
% hObject    handle to Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global boardValue;
global robot;
global realRobot;
global eStop;
    boardValue = 1;
    if realRobot == 1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);

        % Setting values and sending to the robot
        areal = 1; % acceleration, doesnt do anything...
        vreal = 1; % velocity slowest to fastest = 0 to 1.

        cute_multi_joint_msg.Vel = vreal;
        cute_multi_joint_msg.Acc = areal;
        while eStop == 1
        drawnow;
        end
        cute_multi_joint_msg.JointStates = [0 0 0 0 0 0 0];
        cute_multi_joint_client.call(cute_multi_joint_msg);
    else
        robot.model.plot([0 0 0 0 0 0 0])
    end
% --- Executes on button press in EmergencyStop.
function EmergencyStop_Callback(hObject, eventdata, handles)
% hObject    handle to EmergencyStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global eStop
eStop = get(hObject,'Value')
% Hint: get(hObject,'Value') returns toggle state of EmergencyStop


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1



function XInput_Callback(hObject, eventdata, handles)
% hObject    handle to XInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xInput;
xInput = str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of XInput as text
%        str2double(get(hObject,'String')) returns contents of XInput as a double


% --- Executes during object creation, after setting all properties.
function XInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to XInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function YInput_Callback(hObject, eventdata, handles)
% hObject    handle to YInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global yInput;
yInput = str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of YInput as text
%        str2double(get(hObject,'String')) returns contents of YInput as a double


% --- Executes during object creation, after setting all properties.
function YInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ZInput_Callback(hObject, eventdata, handles)
% hObject    handle to ZInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global zInput;
zInput = str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of ZInput as text
%        str2double(get(hObject,'String')) returns contents of ZInput as a double


% --- Executes during object creation, after setting all properties.
function ZInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ZInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function RollInput_Callback(hObject, eventdata, handles)
% hObject    handle to RollInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global rollInput;
rollInput = str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of RollInput as text
%        str2double(get(hObject,'String')) returns contents of RollInput as a double


% --- Executes during object creation, after setting all properties.
function RollInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RollInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function PitchInput_Callback(hObject, eventdata, handles)
% hObject    handle to PitchInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global pitchInput;
pitchInput = str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of PitchInput as text
%        str2double(get(hObject,'String')) returns contents of PitchInput as a double


% --- Executes during object creation, after setting all properties.
function PitchInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PitchInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function YawInput_Callback(hObject, eventdata, handles)
% hObject    handle to YawInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global yawInput;
yawInput = str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of YawInput as text
%        str2double(get(hObject,'String')) returns contents of YawInput as a double


% --- Executes during object creation, after setting all properties.
function YawInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YawInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in CustomCartesian.
function CustomCartesian_Callback(hObject, eventdata, handles)
% hObject    handle to CustomCartesian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;
global joint7;
global xInput;
global yInput;
global zInput;
global rollInput;
global pitchInput;
global yawInput;
global robot;
global eStop;
global realRobot;
global qCurrent;
global steps;
global s;
    xyzColumn = [xInput;yInput;zInput]
    endeffectorInput = [rpy2r(rollInput,pitchInput,yawInput) xyzColumn;zeros(1,3) 1]
    qNext = robot.model.ikcon(endeffectorInput)
%get the current joint state of real robot or simulation
    if realRobot ==1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
        stateSub = rossubscriber('/joint_states');
        receive(stateSub,2)
        msg = stateSub.LatestMessage;
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7)'
    else
        qCurrent = robot.model.getpos()
    end
    %get the trajectory
        for i=1:steps
            % Use trapezoidal velocity interpolation to find the next joint
            % angle.
            qTry = (1-s(i))*qCurrent + s(i)*qNext
            qMatrix(i,:) = qTry;
        end
    %plot the trajectory in simulation
        for i=1:steps
            % Interrupt While loop that executes upon Emergency Stop toggle
            % button being pressed. the drawnow allows interruption, and once
            % the Emergency Stop toggle button is depressed, the movement
            % continues.
            while eStop == 1
            drawnow;
            end
            robot.model.plot(qMatrix(i,:));
            joint1 = deg2rad(qMatrix(i,1));
            joint2 = deg2rad(qMatrix(i,2));
            joint3 = deg2rad(qMatrix(i,3));
            joint4 = deg2rad(qMatrix(i,4));
            joint5 = deg2rad(qMatrix(i,5));
            joint6 = deg2rad(qMatrix(i,6));
            joint7 = deg2rad(qMatrix(i,7));
            handles.Joint1Box.String = joint1; %for edit box
            handles.Joint2Box.String = joint2; %for edit box
            handles.Joint3Box.String = joint3; %for edit box
            handles.Joint4Box.String = joint4; %for edit box
            handles.Joint5Box.String = joint5; %for edit box
            handles.Joint6Box.String = joint6; %for edit box
            handles.Joint7Box.String = joint7; %for edit box 
            
            endeffector = robot.model.fkine(qMatrix(i,:))
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
    %plot the trajectory in real robot
        if realRobot == 1
            for i=1:steps
            while eStop == 1
            drawnow;
            end
            cute_multi_joint_msg.JointStates = qMatrix(i,:);
            cute_multi_joint_client.call(cute_multi_joint_msg);
            joint1 = deg2rad(qMatrix(i,1));
            joint2 = deg2rad(qMatrix(i,2));
            joint3 = deg2rad(qMatrix(i,3));
            joint4 = deg2rad(qMatrix(i,4));
            joint5 = deg2rad(qMatrix(i,5));
            joint6 = deg2rad(qMatrix(i,6));
            joint7 = deg2rad(qMatrix(i,7));
            handles.Joint1Box.String = joint1; %for edit box
            handles.Joint2Box.String = joint2; %for edit box
            handles.Joint3Box.String = joint3; %for edit box
            handles.Joint4Box.String = joint4; %for edit box
            handles.Joint5Box.String = joint5; %for edit box
            handles.Joint6Box.String = joint6; %for edit box
            handles.Joint7Box.String = joint7; %for edit box 
            
            endeffector = robot.model.fkine(qMatrix(i,:))
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

% --- Executes on slider movement.
function Joint1_Callback(hObject, eventdata, handles)
% hObject    handle to Joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint1;
global joint1Min;
global joint1Max;
set(hObject, 'min', joint1Min);
set(hObject, 'max', joint1Max);
set(hObject, 'SliderStep', [1/(joint1Max-joint1Min) , 10/(joint1Max-joint1Min)]);
joint1 = get(hObject,'Value')
handles.Joint1Box.String = joint1; %for edit box
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Joint1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Joint2_Callback(hObject, eventdata, handles)
% hObject    handle to Joint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint2;
global joint2Min;
global joint2Max;
set(hObject, 'min', joint2Min);
set(hObject, 'max', joint2Max);
set(hObject, 'SliderStep', [1/(joint2Max-joint2Min) , 10/(joint2Max-joint2Min)]);
joint2 = get(hObject,'Value')
handles.Joint2Box.String = joint2; %for edit box

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Joint2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Joint3_Callback(hObject, eventdata, handles)
% hObject    handle to Joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint3;
global joint3Min;
global joint3Max;
set(hObject, 'min', joint3Min);
set(hObject, 'max', joint3Max);
set(hObject, 'SliderStep', [1/(joint3Max-joint3Min) , 10/(joint3Max-joint3Min)]);
joint3 = get(hObject,'Value')
handles.Joint3Box.String = joint3; %for edit box

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Joint3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Joint4_Callback(hObject, eventdata, handles)
% hObject    handle to Joint4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint4;
global joint4Min;
global joint4Max;
set(hObject, 'min', joint4Min);
set(hObject, 'max', joint4Max);
set(hObject, 'SliderStep', [1/(joint4Max-joint4Min) , 10/(joint4Max-joint4Min)]);
joint4 = get(hObject,'Value')
handles.Joint4Box.String = joint4; %for edit box

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Joint4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Joint5_Callback(hObject, eventdata, handles)
% hObject    handle to Joint5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint5;
global joint5Min;
global joint5Max;
set(hObject, 'min', joint5Min);
set(hObject, 'max', joint5Max);
set(hObject, 'SliderStep', [1/(joint5Max-joint5Min) , 10/(joint5Max-joint5Min)]);
joint5 = get(hObject,'Value')
handles.Joint5Box.String = joint5; %for edit box

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Joint5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Joint6_Callback(hObject, eventdata, handles)
% hObject    handle to Joint6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint6;
global joint6Min;
global joint6Max;
set(hObject, 'min', joint6Min);
set(hObject, 'max', joint6Max);
set(hObject, 'SliderStep', [1/(joint6Max-joint6Min) , 10/(joint6Max-joint6Min)]);
joint6 = get(hObject,'Value')
handles.Joint6Box.String = joint6; %for edit box

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Joint6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Joint7_Callback(hObject, eventdata, handles)
% hObject    handle to Joint7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint7;
global joint7Min;
global joint7Max;
set(hObject, 'min', joint7Min);
set(hObject, 'max', joint7Max);
set(hObject, 'SliderStep', [1/(joint7Max-joint7Min) , 10/(joint7Max-joint7Min)]);
joint7 = get(hObject,'Value')
handles.Joint7Box.String = joint7; %for edit box

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Joint7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in CustomJoints.
function CustomJoints_Callback(hObject, eventdata, handles)
% hObject    handle to CustomJoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot;
global eStop;
global realRobot;
global qCurrent;
global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;
global joint7;
global xInput;
global yInput;
global zInput;
global rollInput;
global pitchInput;
global yawInput;
global steps;
global s;
q = [joint1 joint2 joint3 joint4 joint5 joint6 joint7];
%get the current joint state of real robot or simulation
if realRobot ==1
    cute_multi_joint_client = rossvcclient('/cute_multi_joint');
    cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
    stateSub = rossubscriber('/joint_states');
    receive(stateSub,2)
    msg = stateSub.LatestMessage;
    currentJointAngles = msg.Position;
    qCurrent = currentJointAngles(1:7)'
else
    qCurrent = robot.model.getpos()
end
%get the trajectory
    for i=1:steps
        % Use trapezoidal velocity interpolation to find the next joint
        % angle.
        qTry = (1-s(i))*qCurrent + s(i)*q
        qMatrix(i,:) = qTry;
    end
%plot the trajectory in simulation
    for i=1:steps
        % Interrupt While loop that executes upon Emergency Stop toggle
        % button being pressed. the drawnow allows interruption, and once
        % the Emergency Stop toggle button is depressed, the movement
        % continues.
        while eStop == 1
        drawnow;
        end
        robot.model.plot(qMatrix(i,:));
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        endeffector = robot.model.fkine(qMatrix(i,:))
        
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
%plot the trajectory in real robot
    if realRobot == 1
        for i=1:steps
        while eStop == 1
        drawnow;
        end
        cute_multi_joint_msg.JointStates = qMatrix(i,:);
        cute_multi_joint_client.call(cute_multi_joint_msg);
        joint1 = deg2rad(qMatrix(i,1));
        joint2 = deg2rad(qMatrix(i,2));
        joint3 = deg2rad(qMatrix(i,3));
        joint4 = deg2rad(qMatrix(i,4));
        joint5 = deg2rad(qMatrix(i,5));
        joint6 = deg2rad(qMatrix(i,6));
        joint7 = deg2rad(qMatrix(i,7));
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box 
        
        endeffector = robot.model.fkine(qMatrix(i,:))
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


function Joint1Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint1Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint1
global joint1Min
global joint1Max

    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(joint1Min) ' and ' num2str(joint1Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = '0';
        joint1 = 0;
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < joint1Min || joint1Max < manualjoint) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = '0';
       joint1 = 0;
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    joint1 = deg2rad(manualjoint)
% Hints: get(hObject,'String') returns contents of Joint1Box as text
%        str2double(get(hObject,'String')) returns contents of Joint1Box as a double


% --- Executes during object creation, after setting all properties.
function Joint1Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint1Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Joint2Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint2Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint2
global joint2Min
global joint2Max

    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(joint2Min) ' and ' num2str(joint2Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = '0';
        joint2 = 0;
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < joint2Min || joint2Max < manualjoint) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = '0';
       joint2 = 0;
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    joint2 = deg2rad(manualjoint)
% Hints: get(hObject,'String') returns contents of Joint2Box as text
%        str2double(get(hObject,'String')) returns contents of Joint2Box as a double


% --- Executes during object creation, after setting all properties.
function Joint2Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint2Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Joint3Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint3Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint3
global joint3Min
global joint3Max

    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(joint3Min) ' and ' num2str(joint3Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = '0';
        joint3 = 0;
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < joint3Min || joint3Max < manualjoint) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = '0';
       joint3 = 0;
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    joint3 = deg2rad(manualjoint)
% Hints: get(hObject,'String') returns contents of Joint3Box as text
%        str2double(get(hObject,'String')) returns contents of Joint3Box as a double


% --- Executes during object creation, after setting all properties.
function Joint3Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint3Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Joint4Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint4Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint4
global joint4Min
global joint4Max

    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(joint4Min) ' and ' num2str(joint4Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = '0';
        joint4 = 0;
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < joint4Min || joint4Max < manualjoint) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = '0';
       joint4 = 0;
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    joint4 = deg2rad(manualjoint)
% Hints: get(hObject,'String') returns contents of Joint4Box as text
%        str2double(get(hObject,'String')) returns contents of Joint4Box as a double


% --- Executes during object creation, after setting all properties.
function Joint4Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint4Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Joint5Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint5Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint5
global joint5Min
global joint5Max

    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(joint5Min) ' and ' num2str(joint5Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = '0';
        joint5 = 0;
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < joint5Min || joint5Max < manualjoint) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = '0';
       joint5 = 0;
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    joint5 = deg2rad(manualjoint)
% Hints: get(hObject,'String') returns contents of Joint5Box as text
%        str2double(get(hObject,'String')) returns contents of Joint5Box as a double


% --- Executes during object creation, after setting all properties.
function Joint5Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint5Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Joint6Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint6Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint6
global joint6Min
global joint6Max

    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(joint6Min) ' and ' num2str(joint6Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = '0';
        joint6 = 0;
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < joint6Min || joint6Max < manualjoint) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = '0';
       joint6 = 0;
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    joint6 = deg2rad(manualjoint)
% Hints: get(hObject,'String') returns contents of Joint6Box as text
%        str2double(get(hObject,'String')) returns contents of Joint6Box as a double


% --- Executes during object creation, after setting all properties.
function Joint6Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint6Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Joint7Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint7Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global joint7
global joint7Min
global joint7Max

    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(joint7Min) ' and ' num2str(joint7Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.ManualPositionValue.String = '0';
        joint7 = 0;
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < joint7Min || joint7Max < manualjoint) 
       uiwait(warndlg(message));
       handles.ManualPositionValue.String = '0';
       joint7 = 0;
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    joint7 = deg2rad(manualjoint)
% Hints: get(hObject,'String') returns contents of Joint7Box as text
%        str2double(get(hObject,'String')) returns contents of Joint7Box as a double


% --- Executes during object creation, after setting all properties.
function Joint7Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint7Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
