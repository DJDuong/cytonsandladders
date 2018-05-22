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

% Last Modified by GUIDE v2.5 22-May-2018 19:53:20

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
    global realistic;
    global eStop;
    global boardValue;
    global boardValueMin;
    global boardValueMax;
    global dieValueMin;
    global dieValueMax;
    global qCurrent;
    global realRobot;
    global rosIP;
    %cells to be used in board algorithm.
    %tim, maybe look into creating an array that can map xyz values to the
    %cell locations. there is a map function on matlab.
    global pitstop;
    global cell1; 
    global cell2; 
    global cell3; 
    global cell4; 
    global cell5; 
    global cell6;  
    global cell7; 
    global cell8; 
    global cell9; 
    global cell10; 
    global cell11; 
    global cell12; 
    global cell13; 
    global cell14; 
    global cell15; 
    global cell16; 
    global cell17; 
    global cell18;  
    global cell19; 
    global cell20; 
    global cell21; 
    global cell22; 
    global cell23; 
    global cell24; 
    global cell25; 
    global cell26; 
    global cell27; 
    global cell28; 
    global cell29; 
    global cell30; 
    global boardXYZMatrix;
    global boardRPYMatrix;
    global joint1;
    global joint2;
    global joint3;
    global joint4;
    global joint5;
    global joint6;
    global joint7;
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

    
    %testing xyz coordinates assigned to cell1 and cell2.
    cell1 = xlsread('celltransformsyConstant.xlsx', 'B1:E4');
    cell2 = xlsread('celltransformsyConstant.xlsx', 'B5:E8');
    cell3 = xlsread('celltransformsyConstant.xlsx', 'B9:E12');
    cell4 = xlsread('celltransformsyConstant.xlsx', 'B13:E16');
    cell5 = xlsread('celltransformsyConstant.xlsx', 'B17:E20');
    cell6 = xlsread('celltransformsyConstant.xlsx', 'B21:E24');
    cell7 = xlsread('celltransformsyConstant.xlsx', 'B25:E28');
    cell8 = xlsread('celltransformsyConstant.xlsx', 'B29:E32');
    cell9 = xlsread('celltransformsyConstant.xlsx', 'B33:E36');
    cell10 = xlsread('celltransformsyConstant.xlsx', 'B37:E40');
    cell11 = xlsread('celltransformsyConstant.xlsx', 'B41:E44');
    cell12 = xlsread('celltransformsyConstant.xlsx', 'B45:E48');
    cell13 = xlsread('celltransformsyConstant.xlsx', 'B49:E52');
    cell14 = xlsread('celltransformsyConstant.xlsx', 'B53:E56');
    cell15 = xlsread('celltransformsyConstant.xlsx', 'B57:E60');
    cell16 = xlsread('celltransformsyConstant.xlsx', 'B61:E64');
    cell17 = xlsread('celltransformsyConstant.xlsx', 'B65:E68');
    cell18 = xlsread('celltransformsyConstant.xlsx', 'B69:E72');
    cell19 = xlsread('celltransformsyConstant.xlsx', 'B73:E76');
    cell20 = xlsread('celltransformsyConstant.xlsx', 'B77:E80');
    cell21 = xlsread('celltransformsyConstant.xlsx', 'B81:E84');
    cell22 = xlsread('celltransformsyConstant.xlsx', 'B85:E88');
    cell23 = xlsread('celltransformsyConstant.xlsx', 'B89:E92');
    cell24 = xlsread('celltransformsyConstant.xlsx', 'B93:E96');
    cell25 = xlsread('celltransformsyConstant.xlsx', 'B97:E100');
    cell26 = xlsread('celltransformsyConstant.xlsx', 'B101:E104');
    cell27 = xlsread('celltransformsyConstant.xlsx', 'B105:E108');
    cell28 = xlsread('celltransformsyConstant.xlsx', 'B109:E112');
    cell29 = xlsread('celltransformsyConstant.xlsx', 'B113:E116');
    cell30 = xlsread('celltransformsyConstant.xlsx', 'B117:E120');
    %matrix used to allow die value to increment element position.
    boardXYZMatrix = [cell1(1:3,4)'; cell2(1:3,4)'; cell3(1:3,4)'; cell4(1:3,4)'; ...
        cell5(1:3,4)'; cell6(1:3,4)'; cell7(1:3,4)'; cell8(1:3,4)';...
        cell9(1:3,4)'; cell10(1:3,4)'; cell11(1:3,4)'; cell12(1:3,4)';...
        cell13(1:3,4)'; cell14(1:3,4)'; cell15(1:3,4)'; cell16(1:3,4)';...
        cell17(1:3,4)'; cell18(1:3,4)'; cell19(1:3,4)'; cell20(1:3,4)';...
        cell21(1:3,4)'; cell22(1:3,4)'; cell23(1:3,4)'; cell24(1:3,4)';...
        cell25(1:3,4)'; cell26(1:3,4)'; cell27(1:3,4)'; cell28(1:3,4)';...
        cell29(1:3,4)'; cell30(1:3,4)']
    boardRPYMatrix = [tr2rpy(cell1);tr2rpy(cell2);tr2rpy(cell3);tr2rpy(cell4);...
        tr2rpy(cell5);tr2rpy(cell6);tr2rpy(cell7);tr2rpy(cell8);tr2rpy(cell9);...
        tr2rpy(cell10);tr2rpy(cell11);tr2rpy(cell12);tr2rpy(cell13);tr2rpy(cell14);...
        tr2rpy(cell15);tr2rpy(cell16);tr2rpy(cell17);tr2rpy(cell18);tr2rpy(cell19);...
        tr2rpy(cell20);tr2rpy(cell21);tr2rpy(cell22);tr2rpy(cell23);tr2rpy(cell24);...
        tr2rpy(cell25);tr2rpy(cell26);tr2rpy(cell27);tr2rpy(cell28);tr2rpy(cell29);...
        tr2rpy(cell30)]
    pitstop = [-0.1181 0.3007 -0.1074 1.1351 -0.1733 0.8866 1.7242]
    joint1 = 0;
    joint2 = 0;
    joint3 = 0;
    joint4 = 0;
    joint5 = 0;
    joint6 = 0;
    joint7 = 0;
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
    else
            % show cyton at pitstop angle state
        robot.model.plot(pitstop);
        qCurrent = robot.model.getpos();
        joint1 = qCurrent(1)
        joint2 = qCurrent(2)
        joint3 = qCurrent(3)
        joint4 = qCurrent(4)
        joint5 = qCurrent(5)
        joint6 = qCurrent(6)
        handles.Joint1Box.String = joint1; %for edit box
        handles.Joint2Box.String = joint2; %for edit box
        handles.Joint3Box.String = joint3; %for edit box
        handles.Joint4Box.String = joint4; %for edit box
        handles.Joint5Box.String = joint5; %for edit box
        handles.Joint6Box.String = joint6; %for edit box
        handles.Joint7Box.String = joint7; %for edit box
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
global boardXYZMatrix;
global boardRPYMatrix;
global qCurrent;
global realRobot;
global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;
global joint7;
    boardValue;
    boardValue = dieValue + boardValue;
    handles.ManualPositionValue.String = boardValue

    % This is where the trajectory operation should occur.
    if realRobot == 1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
        stateSub = rossubscriber('/joint_states');
        receive(stateSub,2)
        msg = stateSub.LatestMessage;
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
        trCurrent = robot.model.fkine(qCurrent);
        cellXYZCurrent = trCurrent(1:3,4)';
        cellRPYCurrent = tr2rpy(trCurrent);
        % Setting values and sending to the robot
        startPos = cellXYZCurrent;
        endPos = boardXYZMatrix(boardValue,:)
        startRPY = cellRPYCurrent;
        endRPY = boardRPYMatrix(boardValue,:)
        qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
        robot.model.plot(qMatrix)
        areal = 1; % acceleration, doesnt do anything...
        vreal = 1; % velocity slowest to fastest = 0 to 1.

        cute_multi_joint_msg.Vel = vreal;
        cute_multi_joint_msg.Acc = areal;
        [rows,columns] = size(qMatrix)
        for i = 1:rows
            while eStop == 1
            drawnow;
            end
            cute_multi_joint_msg.JointStates = qMatrix(i,:);
            cute_multi_joint_client.call(cute_multi_joint_msg);
            joint1 = deg2rad(qMatrix(i,1))
            joint2 = deg2rad(qMatrix(i,2))
            joint3 = deg2rad(qMatrix(i,3))
            joint4 = deg2rad(qMatrix(i,4))
            joint5 = deg2rad(qMatrix(i,5))
            joint6 = deg2rad(qMatrix(i,6))
            joint7 = deg2rad(qMatrix(i,7))
            handles.Joint1Box.String = joint1; %for edit box
            handles.Joint2Box.String = joint2; %for edit box
            handles.Joint3Box.String = joint3; %for edit box
            handles.Joint4Box.String = joint4; %for edit box
            handles.Joint5Box.String = joint5; %for edit box
            handles.Joint6Box.String = joint6; %for edit box
            handles.Joint7Box.String = joint7; %for edit box
        end
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
    else
        qCurrent = robot.model.getpos();
        trCurrent = robot.model.fkine(qCurrent);
        cellXYZCurrent = trCurrent(1:3,4)';
        cellRPYCurrent = tr2rpy(trCurrent);
        startPos = cellXYZCurrent;
        endPos = boardXYZMatrix(boardValue,:)
        startRPY = cellRPYCurrent;
        endRPY = boardRPYMatrix(boardValue,:)
        qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
        [rows,columns] = size(qMatrix)
            for i = 1:rows
                % Interrupt While loop that executes upon Emergency Stop toggle
                % button being pressed. the drawnow allows interruption, and once
                % the Emergency Stop toggle button is depressed, the movement
                % continues.
                while eStop == 1
                    drawnow;
                end
                robot.model.plot(qMatrix(i,:))
                joint1 = deg2rad(qMatrix(i,1))
                joint2 = deg2rad(qMatrix(i,2))
                joint3 = deg2rad(qMatrix(i,3))
                joint4 = deg2rad(qMatrix(i,4))
                joint5 = deg2rad(qMatrix(i,5))
                joint6 = deg2rad(qMatrix(i,6))
                joint7 = deg2rad(qMatrix(i,7))
                handles.Joint1Box.String = joint1; %for edit box
                handles.Joint2Box.String = joint2; %for edit box
                handles.Joint3Box.String = joint3; %for edit box
                handles.Joint4Box.String = joint4; %for edit box
                handles.Joint5Box.String = joint5; %for edit box
                handles.Joint6Box.String = joint6; %for edit box
                handles.Joint7Box.String = joint7; %for edit box
            end
        qCurrent = robot.model.fkine(robot.model.getpos());
       
    end

    % --- Executes on button press in MoveByPosition.
function MoveByPosition_Callback(hObject, eventdata, handles)
% hObject    handle to MoveByPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global boardValue;
global robot;
global eStop;
global boardXYZMatrix;
global boardRPYMatrix;
global realRobot;
global qCurrent;
global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;
global joint7;
boardValue
    % This is where the trajectory planning should occur.
    if realRobot == 1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
        stateSub = rossubscriber('/joint_states');
        receive(stateSub,2)
        msg = stateSub.LatestMessage;
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
        trCurrent = robot.model.fkine(qCurrent);
        cellXYZCurrent = trCurrent(1:3,4)';
        cellRPYCurrent = tr2rpy(trCurrent);    
        % Setting values and sending to the robot
        startPos = cellXYZCurrent;
        endPos = boardXYZMatrix(boardValue,:)
        startRPY = cellRPYCurrent;
        endRPY = boardRPYMatrix(boardValue,:)
        qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
        robot.model.plot(qMatrix);
        areal = 1; % acceleration, doesnt do anything...
        vreal = 1; % velocity slowest to fastest = 0 to 1.

        cute_multi_joint_msg.Vel = vreal;
        cute_multi_joint_msg.Acc = areal;
        [rows,columns] = size(qMatrix);
        for i = 1:rows
            while eStop == 1
            drawnow;
            end
            cute_multi_joint_msg.JointStates = qMatrix(i,:);
            cute_multi_joint_client.call(cute_multi_joint_msg);
            joint1 = deg2rad(qMatrix(i,1))
            joint2 = deg2rad(qMatrix(i,2))
            joint3 = deg2rad(qMatrix(i,3))
            joint4 = deg2rad(qMatrix(i,4))
            joint5 = deg2rad(qMatrix(i,5))
            joint6 = deg2rad(qMatrix(i,6))
            joint7 = deg2rad(qMatrix(i,7))
            handles.Joint1Box.String = joint1; %for edit box
            handles.Joint2Box.String = joint2; %for edit box
            handles.Joint3Box.String = joint3; %for edit box
            handles.Joint4Box.String = joint4; %for edit box
            handles.Joint5Box.String = joint5; %for edit box
            handles.Joint6Box.String = joint6; %for edit box
            handles.Joint7Box.String = joint7; %for edit box
        end
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);

    else
        qCurrent = robot.model.getpos();
        trCurrent = robot.model.fkine(qCurrent);
        cellXYZCurrent = trCurrent(1:3,4)';
        cellRPYCurrent = tr2rpy(trCurrent);
        startPos = cellXYZCurrent
        endPos = boardXYZMatrix(boardValue,:)
        startRPY = cellRPYCurrent
        endRPY = boardRPYMatrix(boardValue,:)
        qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
        [rows,columns] = size(qMatrix);
            for i = 1:rows
                % Interrupt While loop that executes upon Emergency Stop toggle
                % button being pressed. the drawnow allows interruption, and once
                % the Emergency Stop toggle button is depressed, the movement
                % continues.
                while eStop == 1
                    drawnow;
                end
                robot.model.plot(qMatrix(i,:))
                joint1 = deg2rad(qMatrix(i,1))
                joint2 = deg2rad(qMatrix(i,2))
                joint3 = deg2rad(qMatrix(i,3))
                joint4 = deg2rad(qMatrix(i,4))
                joint5 = deg2rad(qMatrix(i,5))
                joint6 = deg2rad(qMatrix(i,6))
                joint7 = deg2rad(qMatrix(i,7))
                handles.Joint1Box.String = joint1; %for edit box
                handles.Joint2Box.String = joint2; %for edit box
                handles.Joint3Box.String = joint3; %for edit box
                handles.Joint4Box.String = joint4; %for edit box
                handles.Joint5Box.String = joint5; %for edit box
                handles.Joint6Box.String = joint6; %for edit box
                handles.Joint7Box.String = joint7; %for edit box
            end
        qCurrent = robot.model.fkine(robot.model.getpos());

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
    xyzNext = [xInput yInput zInput]
    rpyNext = [rollInput pitchInput yawInput]
    qCurrent = robot.model.getpos();
    trCurrent = robot.model.fkine(qCurrent);
    cellXYZCurrent = trCurrent(1:3,4)';
    cellRPYCurrent = tr2rpy(trCurrent);
    startPos = cellXYZCurrent
    endPos = xyzNext
    startRPY = cellRPYCurrent
    endRPY = rpyNext
    qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
    [rows,columns] = size(qMatrix);
        for i = 1:rows
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
        end
    qCurrent = robot.model.fkine(robot.model.getpos())
    if realRobot == 1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
        stateSub = rossubscriber('/joint_states');
        receive(stateSub,2)
        msg = stateSub.LatestMessage;
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
        trCurrent = robot.model.fkine(qCurrent);
        cellXYZCurrent = trCurrent(1:3,4)';
        cellRPYCurrent = tr2rpy(trCurrent);    
        % Setting values and sending to the robot
        startPos = cellXYZCurrent;
        endPos = xyzNext;
        startRPY = cellRPYCurrent;
        endRPY = rpyNext;
        qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
        robot.model.plot(qMatrix);
        areal = 1; % acceleration, doesnt do anything...
        vreal = 1; % velocity slowest to fastest = 0 to 1.

        cute_multi_joint_msg.Vel = vreal;
        cute_multi_joint_msg.Acc = areal;
        [rows,columns] = size(qMatrix);
        for i = 1:rows
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
        end
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
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
global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;
global joint7;
global realRobot;
global robot;
global eStop;
global qCurrent;
    q = [joint1 joint2 joint3 joint4 joint5 joint6 joint7];
    trNext = robot.model.fkine(q);
    xyzNext = trNext(1:3,4)';
    rpyNext = tr2rpy(trNext);
    qCurrent = robot.model.getpos();
    trCurrent = robot.model.fkine(qCurrent);
    cellXYZCurrent = trCurrent(1:3,4)';
    cellRPYCurrent = tr2rpy(trCurrent);
    startPos = cellXYZCurrent;
    endPos = xyzNext;
    startRPY = cellRPYCurrent;
    endRPY = rpyNext;
    qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
    [rows,columns] = size(qMatrix);
        for i = 1:rows
            % Interrupt While loop that executes upon Emergency Stop toggle
            % button being pressed. the drawnow allows interruption, and once
            % the Emergency Stop toggle button is depressed, the movement
            % continues.
            while eStop == 1
                drawnow;
            end
            robot.model.plot(qMatrix(i,:))
            joint1 = deg2rad(qMatrix(i,1))
            joint2 = deg2rad(qMatrix(i,2))
            joint3 = deg2rad(qMatrix(i,3))
            joint4 = deg2rad(qMatrix(i,4))
            joint5 = deg2rad(qMatrix(i,5))
            joint6 = deg2rad(qMatrix(i,6))
            joint7 = deg2rad(qMatrix(i,7))
            handles.Joint1Box.String = joint1; %for edit box
            handles.Joint2Box.String = joint2; %for edit box
            handles.Joint3Box.String = joint3; %for edit box
            handles.Joint4Box.String = joint4; %for edit box
            handles.Joint5Box.String = joint5; %for edit box
            handles.Joint6Box.String = joint6; %for edit box
            handles.Joint7Box.String = joint7; %for edit box
        end
    qCurrent = robot.model.fkine(robot.model.getpos());
    if realRobot == 1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);
        stateSub = rossubscriber('/joint_states');
        receive(stateSub,2)
        msg = stateSub.LatestMessage;
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
        trCurrent = robot.model.fkine(qCurrent);
        cellXYZCurrent = trCurrent(1:3,4)';
        cellRPYCurrent = tr2rpy(trCurrent);    
        % Setting values and sending to the robot
        startPos = cellXYZCurrent;
        endPos = xyzNext;
        startRPY = cellRPYCurrent;
        endRPY = rpyNext;
        qMatrix = RMRC(robot.model, startPos, endPos, startRPY, endRPY);
        robot.model.plot(qMatrix);
        areal = 1; % acceleration, doesnt do anything...
        vreal = 1; % velocity slowest to fastest = 0 to 1.

        cute_multi_joint_msg.Vel = vreal;
        cute_multi_joint_msg.Acc = areal;
        [rows,columns] = size(qMatrix);
        for i = 1:rows
            while eStop == 1
            drawnow;
            end
            cute_multi_joint_msg.JointStates = qMatrix(i,:);
            cute_multi_joint_client.call(cute_multi_joint_msg);
            joint1 = deg2rad(qMatrix(i,1))
            joint2 = deg2rad(qMatrix(i,2))
            joint3 = deg2rad(qMatrix(i,3))
            joint4 = deg2rad(qMatrix(i,4))
            joint5 = deg2rad(qMatrix(i,5))
            joint6 = deg2rad(qMatrix(i,6))
            joint7 = deg2rad(qMatrix(i,7))
            handles.Joint1Box.String = joint1; %for edit box
            handles.Joint2Box.String = joint2; %for edit box
            handles.Joint3Box.String = joint3; %for edit box
            handles.Joint4Box.String = joint4; %for edit box
            handles.Joint5Box.String = joint5; %for edit box
            handles.Joint6Box.String = joint6; %for edit box
            handles.Joint7Box.String = joint7; %for edit box
        end
        currentJointAngles = msg.Position;
        qCurrent = currentJointAngles(1:7);
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



