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

% Last Modified by GUIDE v2.5 18-May-2018 12:24:20

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
    global fps;
    global scale;
    global steps;
    global s;
    global qZero;
    global eStop;
    global boardValue;
    global boardValueMin;
    global boardValueMax;
    global dieValueMin;
    global dieValueMax;
    global realRobot;
    %cells to be used in board algorithm.
    %tim, maybe look into creating an array that can map xyz values to the
    %cell locations. there is a map function on matlab.
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
    global cellCurrent;
    global boardMatrix;
    global boardElement;
    %matrix used to allow die value to increment element position.
    boardMatrix = [cell1; cell2; cell3; cell4; cell5; cell6; cell7; cell8;...
        cell9; cell10; cell11; cell12; cell13; cell14; cell15; cell16;...
        cell17; cell18; cell19; cell20; cell21; cell22; cell23; cell24;...
        cell25; cell26; cell27; cell28; cell29; cell30]
    %testing xyz coordinates assigned to cell1 and cell2.
    cell1 = [-0.15 0.15 0.2]
    cell2 = [-0.10 0.15 0.2]
    cell3 = [-0.05 0.15 0.2]
    cell4 = [0.00 0.15 0.2]
    cell5 = [0.05 0.15 0.2]
    cell6 = [0.10 0.15 0.2]
    boardElement = 1;
    cellCurrent = boardMatrix(boardElement,:)
%     cell3; 
%     cell4; 
%     cell5; 
%     cell6;  
%     cell7; 
%     cell8; 
%     cell9; 
%     cell10; 
%     cell11; 
%     cell12; 
%     cell13; 
%     cell14; 
%     cell15; 
%     cell16; 
%     cell18;  
%     cell19; 
%     cell20; 
%     cell21; 
%     cell22; 
%     cell23; 
%     cell24; 
%     cell25; 
%     cell26; 
%     cell27; 
%     cell28; 
%     cell29; 
%     cell30; 
    boardValue = 1;
    realRobot = 0
    rosIP = '192.168.0.102'
    % Range of dieValue - six faces, 1-6.
    dieValueMin = 1;
    dieValueMax = 6;
    
    % Number of spaces on the board - there are 30 spaces.
    boardValueMin = 1;
    boardValueMax = 30;
    
    % initialise the eStop variable so that the toggle changes it to 1 upon
    % stopping.
    eStop = 0;
    
    % realistic variable = 1 - realistic arms, 2 - simple shape arms, 3 - 
    % stick model arms.
    realistic = 3;          
    
    % handle for objects cyton class
    robot = Cyton();         
    
    % fps variable that sets the frame rate of the arm plot
    fps = 25;       
    
    % scale of the stick and simple shapes models, kept low to ensure the
    % realistic plot can be seen
    scale = 0.3;            
    
    % steps variable for number of steps
    steps = 25;          
    
    % trapezoidal trajectory interpolation technique
    s = lspb(0,1,steps);    
    
    % simple zero joint state for non actuated state.
    qZero = [0 0 0 0 0 0 0];         
    
    % show cyton at zero joint angle state
    robot.model.plot(qZero);
    
    %Tims ply stuff - environment.
    
    
    if realRobot == 1
        % Enabling robot movement and gripper
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
global cellCurrent;
global boardMatrix;
global realRobot;
    boardValue
    boardValue = dieValue + boardValue
    % This is where the trajectory planning should occur.
    
    % This is where the trajectory operation should occur.
    if realRobot == 1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);

        % Setting values and sending to the robot
        startPos = cellCurrent;
        endPos = boardMatrix(boardValue,:)
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
            while eStop == 1
            drawnow;
            end
            cute_multi_joint_msg.JointStates = qMatrix(i,:);
            cute_multi_joint_client.call(cute_multi_joint_msg);
        end
        cellCurrent = boardMatrix(boardValue,:)
        actualCurrent = robot.model.fkine(robot.model.getpos())

    else
        startPos = cellCurrent
        endPos = boardMatrix(boardValue,:)
        rpy = deg2rad([-90 0 0])
        qMatrix = RMRS(robot.model, startPos, endPos, rpy);
        [rows,columns] = size(qMatrix)
            for i = 1:rows
                % Interrupt While loop that executes upon Emergency Stop toggle
                % button being pressed. the drawnow allows interruption, and once
                % the Emergency Stop toggle button is depressed, the movement
                % continues.
                while eStop == 1
                    drawnow;
                end
                i
                robot.model.plot(qMatrix(i,:))
                % This is where the trajectory operation should occur.
            end
        cellCurrent = boardMatrix(boardValue,:)
        poseCurrent = robot.model.fkine(robot.model.getpos());
        xyzCurrent = poseCurrent(1:3,4)

    end

    % --- Executes on button press in MoveByPosition.
function MoveByPosition_Callback(hObject, eventdata, handles)
% hObject    handle to MoveByPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global boardValue;
global robot;
global eStop;
global cellCurrent;
global boardMatrix;
global realRobot;
    moveValue = boardValue;
    % This is where the trajectory planning should occur.
    if realRobot == 1
        cute_multi_joint_client = rossvcclient('/cute_multi_joint');
        cute_multi_joint_msg = rosmessage(cute_multi_joint_client);

        % Setting values and sending to the robot
        startPos = cellCurrent;
        endPos = boardMatrix(boardValue,:)
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
            while eStop == 1
            drawnow;
            end
            cute_multi_joint_msg.JointStates = qMatrix(i,:);
            cute_multi_joint_client.call(cute_multi_joint_msg);
        end
        cellCurrent = boardMatrix(boardValue,:)
        actualCurrent = robot.model.fkine(robot.model.getpos())

    else
        startPos = cellCurrent
        endPos = boardMatrix(boardValue,:)
        rpy = deg2rad([-90 0 0])
        qMatrix = RMRS(robot.model, startPos, endPos, rpy);
        [rows,columns] = size(qMatrix)
            for i = 1:rows
                % Interrupt While loop that executes upon Emergency Stop toggle
                % button being pressed. the drawnow allows interruption, and once
                % the Emergency Stop toggle button is depressed, the movement
                % continues.
                while eStop == 1
                    drawnow;
                end
                i
                robot.model.plot(qMatrix(i,:))
                % This is where the trajectory operation should occur.
            end
        cellCurrent = boardMatrix(boardValue,:)
        actualCurrent = robot.model.fkine(robot.model.getpos())
    end

% --- Executes on button press in Reset.
function Reset_Callback(hObject, eventdata, handles)
% hObject    handle to Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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
