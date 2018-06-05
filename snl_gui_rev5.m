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

% Last Modified by GUIDE v2.5 05-Jun-2018 09:40:54

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
%% ====================BELOW IS RUN BEFORE GUI APPEARS=====================%
    %> global variables where some have default values; initialised before
    % the gui is made visible
    global robot;
    global boardValue;
    global boardValueMin;
    global boardValueMax;
    global dieValueMin;
    global dieValueMax;
    global qZero;
    global realRobot;
    global rosIP;
    global pitstop;
    global steps;
    global s;
    global cellMatrix;
    global pickupMatrix;
    global scale;
    global gripDie;
    global gripPiece;
    global open;
    global trStart;
    global collisioncheck;
    global qlim;
    addpath('cyton')
    addpath('environment')
    addpath('jointangles')
    
    collisioncheck = 1;
    cellMatrix = xlsread('jointangles/celljointAngles.xlsx');
    pickupMatrix = xlsread('jointangles/pickupjointAngles.xlsx');
    pitstop = xlsread('jointangles/pitstop.xlsx');

    % handle for objects cyton class
    robot = Cyton();
    robot.model.base = transl(0,0,0.1185);
    trStart = robot.model.fkine(pickupMatrix(1,:));
    qlim = robot.model.qlim;
    
    steps = 5;
    s = lspb(0,1,steps);
    qZero = [0 0 0 0 0 0 0];
    scale = 0.1;
    gripDie = -0.65;
    gripPiece = -1.3;
    open = 0;
    
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
    
    % initialise the handles.EmergencyStop.Value variable so that the toggle changes it to 1 upon
    % stopping.
    handles.EmergencyStop.Value = 0;       

%% ====================ABOVE IS RUN BEFORE GUI APPEARS=====================%
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
global dieValueMax
handles.ManualDie.String = randi(dieValueMax)
set(handles.GameStatus, 'String','Game Status: Press Play!', 'BackgroundColor', 'w');

function ManualDie_Callback(hObject, eventdata, handles)
% hObject    handle to ManualDie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dieValueMin
global dieValueMax
    message = sprintf(['Input must be an integer, inclusive between ' ...
            num2str(dieValueMin) ' and ' num2str(dieValueMax) '.']);
    % getting the handle object, converting it to a double and setting it to a
    % temporary die value called manualDieValue.
    manualDieValue = str2double(get(hObject,'String'));
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
    handles.ManualDie.String = manualDieValue;
    set(handles.GameStatus, 'String','Game Status: Press Play!', 'BackgroundColor', 'w');

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
global boardValueMin
global boardValueMax

    message = sprintf(['Input must be an integer, inclusive between ' ...
            num2str(boardValueMin) ' and ' num2str(boardValueMax) '.']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualBoardValue = str2double(get(hObject,'String'));
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
    handles.ManualPositionValue.String = manualBoardValue;
    
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

    % --- Executes on button press in MoveByPosition.
function MoveByPosition_Callback(hObject, eventdata, handles)
% hObject    handle to MoveByPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Globals
    global boardValue;
    global robot;
    global realRobot;
    global cellMatrix;
    global pickupMatrix;
    global steps;
    global s;
    global scale;
    global gamePiece;
    global gamePieceVerts;
    global gamePieceVertexCount;
    global collisioncheck;
    
%% Move back with gripped piece
    qNext = cellMatrix(boardValue,:)
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Getting newly input board value
    boardValue = str2num(handles.ManualPositionValue.String)
%% Moving the piece to the next board value
    qNext = cellMatrix(boardValue,:)
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Move piece towards board
    qNext = pickupMatrix(boardValue,:);
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)

% --- Executes on button press in Reset.
function Reset_Callback(hObject, eventdata, handles)
% hObject    handle to Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Globals
    global boardValue;
    global robot;
    global realRobot;
    global cellMatrix;
    global steps;
    global s;
    global pitstop;
    global scale;
    global gamePiece;
    global gamePieceVerts;
    global gamePieceVertexCount;
    global collisioncheck;
    
%% Move back with gripped piece
    qNext = cellMatrix(boardValue,:)
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%% Moving the piece to the pitstop
    Movement(robot, realRobot, s, steps, pitstop, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)

    % --- Executes on button press in Initialise.
function Initialise_Callback(hObject, eventdata, handles)
% hObject    handle to Initialise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Globals
    global robot;
    global realRobot;
    global pitstop;
    global steps;
    global s;
    global scale;
    global rosIP;
    global open;
    global boardValue;
    global cellMatrix;
    global pickupMatrix;
    global gripPiece;
    global gamePiece;
    global gamePieceVerts;
    global gamePieceVertexCount;
    global trStart;
    global collisioncheck;
    global vTotal;
    global fTotal;
    set(handles.GameStatus, 'String','Game Status: Initialising...', 'BackgroundColor', 'w');
%% Setting up environment
    figure(1);  
    axes;
    robot.PlotAndColourRobot();
    hold on
    %Cage
    [fCage,vCage,dataCage] = plyread('environment/Cage5.ply','tri');
    vertexColours = [dataCage.vertex.red, dataCage.vertex.green, dataCage.vertex.blue] / 255;
    cage = trisurf(fCage,vCage(:,1)+ 0,vCage(:,2) + 0, vCage(:,3) +0.01,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    %Board
    [fBoard,vBoard,dataBoard] = plyread('environment/Board2.ply','tri');      
    vertexColours = [dataBoard.vertex.red, dataBoard.vertex.green, dataBoard.vertex.blue] / 255;
    board = trisurf(fBoard,vBoard(:,1)+0,vBoard(:,2) + 0.272, vBoard(:,3) + 0.02,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    %Button
    [fButton,vButton,dataButton] = plyread('environment/button.ply','tri');      
    vertexColours = [dataButton.vertex.red, dataButton.vertex.green, dataButton.vertex.blue] / 255;
    button = trisurf(fButton,vButton(:,1),vButton(:,2)+0.415, vButton(:,3)+0.2,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    %UR3 Jig Plate
    [fPlate,vPlate,dataPlate] = plyread('environment/ur3 jig plate.ply','tri');      
    vertexColours = [dataPlate.vertex.red, dataPlate.vertex.green, dataPlate.vertex.blue] / 255;
    ur3 = trisurf(fPlate,vPlate(:,1)+0,vPlate(:,2)+0, vPlate(:,3)+0.02,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    %Cyton Base
    [fCytonBase,vCytonBase,dataCytonBase] = plyread('environment/cytonbase.ply','tri');      
    vertexColours = [dataCytonBase.vertex.red, dataCytonBase.vertex.green, dataCytonBase.vertex.blue] / 255;
    cytonBase = trisurf(fCytonBase,vCytonBase(:,1)+0,vCytonBase(:,2)+0, vCytonBase(:,3)+0.028,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    %Robot Base
    [fRobotBase,vRobotBase,dataRobotBase] = plyread('environment/RobotBase.ply','tri');      
    vertexColours = [dataRobotBase.vertex.red, dataRobotBase.vertex.green, dataRobotBase.vertex.blue] / 255;
    robotBase = trisurf(fRobotBase,vRobotBase(:,1)+0,vRobotBase(:,2)+0, vRobotBase(:,3)+0.028,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    %Latch 1
    [fLatch1,vLatch1,dataLatch1] = plyread('environment/latch.ply','tri');      
    vertexColours = [dataLatch1.vertex.red, dataLatch1.vertex.green, dataLatch1.vertex.blue] / 255;
    latch1 = trisurf(fLatch1,vLatch1(:,1)+0.15,vLatch1(:,2)+0.175, vLatch1(:,3)+0.033,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
   %Latch 2
    [fLatch2,vLatch2,dataLatch2] = plyread('environment/latch.ply','tri');      
    vertexColours = [dataLatch2.vertex.red, dataLatch2.vertex.green, dataLatch2.vertex.blue] / 255;
    latch2 = trisurf(fLatch2,vLatch2(:,1)-0.15,vLatch2(:,2)+0.175, vLatch2(:,3)+0.033,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
   %IRStand
    [fIRStand,vIRStand,dataIRStand] = plyread('environment/irandstand.ply','tri');      
    vertexColours = [dataIRStand.vertex.red, dataIRStand.vertex.green, dataIRStand.vertex.blue] / 255;
    irStand = trisurf(fIRStand,vIRStand(:,1)+0.15,vIRStand(:,2)-0.08, vIRStand(:,3)+0.033,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
   %IRBlock
    [fIRBlock,vIRBlock,dataIRBlock] = plyread('environment/irstand2.ply','tri');      
    vertexColours = [dataIRBlock.vertex.red, dataIRBlock.vertex.green, dataIRBlock.vertex.blue] / 255;
    irBlock = trisurf(fIRBlock,vIRBlock(:,1)-0.15,vIRBlock(:,2)-0.08, vIRBlock(:,3)+0.033,...
       'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
   %image is 260mm x 175mm
    img = imread('environment/Board.jpg');
    xImage = [-0.13 0.13; -0.13 0.13];   %# The x data for the image corners
    yImage = [0.368 0.368; 0.368 0.368];             %# The y data for the image corners
    zImage = [0.25 0.25; 0.05 0.05];   %# The z data for the image corners
    surf(xImage,yImage,zImage,...    %# Plot the surface
     'CData',img,...
     'FaceColor','texturemap');
    %Game Piece
    [f,v,data] = plyread('environment/gamePiece.ply','tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    gamePieceVertexCount = size(v,1);
    gamePieceVerts = v;
    gamePiece = trisurf(f,gamePieceVerts(:,1)+ 0,...
    gamePieceVerts(:,2) + 0,...
    gamePieceVerts(:,3) + 0,...
    'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    updatedPoints = [trStart * [gamePieceVerts,ones(gamePieceVertexCount,1)]']';  
        gamePiece.Vertices = updatedPoints(:,1:3);
%     fTotal = {fCage}; 
%     vTotal = {cage.Vertices};
    fTotal = {board.Faces}; 
    vTotal = {board.Vertices};
    fTotal{end+1} = latch1.Faces; 
    vTotal{end+1} = latch1.Vertices;
    fTotal{end+1} = latch2.Faces; 
    vTotal{end+1} = latch2.Vertices;
    fTotal{end+1} = irStand.Faces; 
    vTotal{end+1} = irStand.Vertices;
    fTotal{end+1} = irBlock.Faces; 
    vTotal{end+1} = irBlock.Vertices;
    
    camlight;
    handles.ManualPositionValue.String = 1;

%% Energise Robot and Open Gripper
%get the current joint state of real robot or simulation
if realRobot == 1
    % Open PuTTy, enter 192.168.0.100 and connect.
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
    % If successful, green text saying "You can start planning now!" will
    % appear.
    
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
        
    %Open the claw
    cute_claw_publisher = rospublisher('/claw_controller/command');
    cute_claw_msg = rosmessage(cute_claw_publisher);
    cute_claw_msg.Data = open; % Values must be between -1.5 (closed) and 0 (open)
    cute_claw_publisher.send(cute_claw_msg);
end
    set(handles.GameStatus, 'String','Game Status: Initialising, press space whilst clicked in...', 'BackgroundColor', 'w');
pause;
    set(handles.GameStatus, 'String','Game Status: Initialising...', 'BackgroundColor', 'w');
%% Move to Pitstop
    qNext = pitstop;
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Move to Start Cell
    qNext = cellMatrix(boardValue,:)
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Move to pick up the piece.
    qNext = pickupMatrix(boardValue,:)
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Close the claw
if realRobot == 1
    cute_claw_publisher = rospublisher('/claw_controller/command');
    cute_claw_msg = rosmessage(cute_claw_publisher);
    cute_claw_msg.Data = gripPiece; % Values must be between -1.5 (closed) and 0 (open)
    cute_claw_publisher.send(cute_claw_msg);
end
    set(handles.GameStatus, 'String','Game Status: Waiting for Die Input...', 'BackgroundColor', 'w');

% --- Executes on button press in Play.
function Play_Callback(hObject, eventdata, handles)
% hObject    handle to Play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Globals
    global boardValue;
    global robot;
    global realRobot;
    global cellMatrix;
    global pickupMatrix;
    global steps;
    global s;
    global pitstop;
    global scale;
    global gamePiece;
    global gamePieceVerts;
    global gamePieceVertexCount;
    global collisioncheck;
    global open;
    global vTotal;
    global fTotal;
    set(handles.GameStatus, 'String','Game Status: Playing...', 'BackgroundColor', 'w');
%% Move back with gripped piece
    qNext = cellMatrix(boardValue,:)
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%% Getting die value and using it to increment the board value
if boardValue + str2num(handles.ManualDie.String) > 30
    remainingBoardSpace = 30 - boardValue
    overshoot = str2num(handles.ManualDie.String) - remainingBoardSpace
    boardValue = 30 - overshoot
else
    boardValue = str2num(handles.ManualDie.String) + boardValue;        
    handles.ManualPositionValue.String = boardValue;
end
    switched = false;
%% Moving the piece to the next board value
    qNext = cellMatrix(boardValue,:)
    CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Move piece towards board
    qNext = pickupMatrix(boardValue,:);
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Check if there is cyton or ladder
    switch boardValue
        case 3
            nextBoardValue = 21;
            handles.ManualPositionValue.String = nextBoardValue;
            switched = true;
        case 14
            nextBoardValue = 27;
            handles.ManualPositionValue.String = nextBoardValue;
            switched = true;
        case 18
            nextBoardValue = 20;
            handles.ManualPositionValue.String = nextBoardValue;
            switched = true;
        case 12
            nextBoardValue = 2;
            handles.ManualPositionValue.String = nextBoardValue;
            switched = true;
        case 26
            nextBoardValue = 13;
            handles.ManualPositionValue.String = nextBoardValue;
            switched = true;
        case 29 
            nextBoardValue = 5;
            handles.ManualPositionValue.String = nextBoardValue;
            switched = true;
    end
%% If there is a cyton or ladder, move piece away from board, then to the next board location and then push towards the board.
    %get the current joint state of real robot or simulation
if switched == true
%% Move back with gripped piece
    qNext = cellMatrix(boardValue,:)
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
    boardValue = nextBoardValue;
%% Moving the piece to the next board value
    qNext = cellMatrix(boardValue,:)
    CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
%% Move piece towards board
    qNext = pickupMatrix(boardValue,:);
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
end
set(handles.GameStatus, 'String','Game Status: Waiting for Die Input...', 'BackgroundColor', 'w');
if boardValue == 30
    set(handles.GameStatus, 'String','Game Status: Winna Winna Chicken Dinna!', 'BackgroundColor', 'g');
    if realRobot == 1
        %Enabling Gripper
        %Setup Variables
        cute_enable_gripper_client = rossvcclient('claw_controller/torque_enable')
        cute_enable_gripper_msg = rosmessage(cute_enable_gripper_client);
        
        %To Enable/Disable the gripper
        cute_enable_gripper_msg.TorqueEnable = true;% false
        cute_enable_gripper_client.call(cute_enable_gripper_msg);

        %Open the claw
        cute_claw_publisher = rospublisher('/claw_controller/command');
        cute_claw_msg = rosmessage(cute_claw_publisher);
        cute_claw_msg.Data = open; % Values must be between -1.5 (closed) and 0 (open)
        cute_claw_publisher.send(cute_claw_msg);
    end  

    %% Move back
    qNext = cellMatrix(boardValue,:);
    % No game piece vertices passed so that the game piece is left at the
    % final position - simulating the gripper opening.
    Movement(robot, realRobot, s, steps, qNext, scale, [], [], [], handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
    %% Move to Pitstop
    qNext = pitstop;
    Movement(robot, realRobot, s, steps, qNext, scale, [], [], [], handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
end

% --- Executes on button press in CustomJoints.
function CustomJoints_Callback(hObject, eventdata, handles)
% hObject    handle to CustomJoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Globals
global robot;
global realRobot;
global steps;
global s;
global scale;
global gamePiece;
global gamePieceVerts;
global gamePieceVertexCount;
global collisioncheck;

%% Move according to input joint angles
    qNext = [str2num(handles.Joint1Box.String) str2num(handles.Joint2Box.String) ...
        str2num(handles.Joint3Box.String) str2num(handles.Joint4Box.String) ...
        str2num(handles.Joint5Box.String) str2num(handles.Joint6Box.String) ...
        str2num(handles.Joint7Box.String)]
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)

 % --- Executes on button press in CustomCartesian.
function CustomCartesian_Callback(hObject, eventdata, handles)
% hObject    handle to CustomCartesian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Globals
global robot;
global realRobot;
global steps;
global s;
global scale;
global gamePiece;
global gamePieceVerts;
global gamePieceVertexCount;
global collisioncheck;

%% Move according to input xyzrpy
    xyzColumn = [str2num(handles.XInput.String); str2num(handles.YInput.String);...
        str2num(handles.ZInput.String)];
    endEffectorInput = [rpy2r(str2num(handles.RollInput.String) ,...
        str2num(handles.PitchInput.String), str2num(handles.YawInput.String))...
        xyzColumn;zeros(1,3) 1];
    qNext = robot.model.ikcon(endEffectorInput);
    Movement(robot, realRobot, s, steps, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, handles)
%     CollisionCheckMovement(robot, realRobot, s, steps, pitstop, qNext, scale, gamePieceVertexCount, gamePieceVerts, gamePiece, collisioncheck, vTotal, fTotal, handles)
   
% --- Executes on button press in EmergencyStop.
function EmergencyStop_Callback(hObject, eventdata, handles)
% hObject    handle to EmergencyStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.EmergencyStop.Value = get(hObject,'Value')
if handles.EmergencyStop.Value == 1
    handles.EmergencyStop.String = 'Emergency Stop: ON, CLICK AGAIN TO RESUME MOVEMENT';
    set(handles.RobotStatus, 'String','Robot Status: EMERGENCY STOPPED', 'BackgroundColor', 'r');
else
    handles.EmergencyStop.String = 'Emergency Stop: Off';
    set(handles.RobotStatus, 'String','Robot Status: Stationary', 'BackgroundColor', 'w');
end

% Hint: get(hObject,'Value') returns toggle state of EmergencyStop

%=========================================================================
%=========================================================================
%===================== BELOW ARE ALL INPUT FUNCTIONS =====================
%=========================================================================
%=========================================================================

function XInput_Callback(hObject, eventdata, handles)
% hObject    handle to XInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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

% --- Executes on slider movement.
function Joint1_Callback(hObject, eventdata, handles)
% hObject    handle to Joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global qlim;
set(hObject, 'min', qlim(1,1));
set(hObject, 'max', qlim(1,2));
set(hObject, 'SliderStep', [0.1/(qlim(1,2)-qlim(1,1)),...
    1/(qlim(1,2)-qlim(1,1))]);
handles.Joint1Box.String = get(hObject,'Value'); %for edit box
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
global qlim;
set(hObject, 'min', qlim(2,1));
set(hObject, 'max', qlim(2,2));
set(hObject, 'SliderStep', [0.1/(qlim(2,2)-qlim(2,1)) , 1/(qlim(2,2)-qlim(2,1))]);
handles.Joint2Box.String = get(hObject,'Value'); %for edit box

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
global qlim;

set(hObject, 'min', qlim(3,1));
set(hObject, 'max', qlim(3,2));
set(hObject, 'SliderStep', [0.1/(qlim(3,2)-qlim(3,1)) , 1/(qlim(3,2)-qlim(3,1))]);
handles.Joint3Box.String = get(hObject,'Value'); %for edit box

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
global qlim;

set(hObject, 'min', qlim(4,1));
set(hObject, 'max', qlim(4,2));
set(hObject, 'SliderStep', [0.1/(qlim(4,2)-qlim(4,1)) , 1/(qlim(4,2)-qlim(4,1))]);
handles.Joint4Box.String = get(hObject,'Value'); %for edit box

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
global qlim;

set(hObject, 'min', qlim(5,1));
set(hObject, 'max', qlim(5,2));
set(hObject, 'SliderStep', [0.1/(qlim(5,2)-qlim(5,1)) , 1/(qlim(5,2)-qlim(5,1))]);
handles.Joint5Box.String = get(hObject,'Value'); %for edit box

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
global qlim;

set(hObject, 'min', qlim(6,1));
set(hObject, 'max', qlim(6,2));
set(hObject, 'SliderStep', [0.1/(qlim(6,2)-qlim(6,1)) , 1/(qlim(6,2)-qlim(6,1))]);
handles.Joint6Box.String = get(hObject,'Value'); %for edit box

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
global qlim;

set(hObject, 'min', qlim(7,1));
set(hObject, 'max', joint7Max);
set(hObject, 'SliderStep', [0.1/(joint7Max-qlim(7,1)) , 1/(joint7Max-qlim(7,1))]);
handles.Joint7Box.String = get(hObject,'Value'); %for edit box

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

function Joint1Box_Callback(hObject, eventdata, handles)
% hObject    handle to Joint1Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global qlim;


    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(qlim(1,1)) ' and ' num2str(qlim(1,2)) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.Joint1Box.String = '0';
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < qlim(1,1) || qlim(1,2) < manualjoint) 
       uiwait(warndlg(message));
       handles.Joint1Box.String = '0';
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    handles.Joint1Box.String = manualjoint
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
global qlim;


    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(qlim(2,1)) ' and ' num2str(qlim(2,2)) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.Joint2Box.String = '0';
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < qlim(2,1) || qlim(2,2) < manualjoint) 
       uiwait(warndlg(message));
       handles.Joint2Box.String = '0';
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    handles.Joint2Box.String = manualjoint
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
global qlim;


    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(qlim(3,1)) ' and ' num2str(qlim(3,2)) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.Joint3Box.String = '0';
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < qlim(3,1) || qlim(3,2) < manualjoint) 
       uiwait(warndlg(message));
       handles.Joint3Box.String = '0';
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    handles.Joint3Box.String = manualjoint
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
global qlim;


    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(qlim(4,1)) ' and ' num2str(qlim(4,2)) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.Joint4Box.String = '0';
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < qlim(4,1) || qlim(4,2) < manualjoint) 
       uiwait(warndlg(message));
       handles.Joint4Box.String = '0';
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    handles.Joint4Box.String = manualjoint
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
global qlim;


    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(qlim(5,1)) ' and ' num2str(qlim(5,2)) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
         handles.Joint5Box.String = '0';
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < qlim(5,1) || qlim(5,2) < manualjoint) 
       uiwait(warndlg(message));
       handles.Joint5Box.String = '0';
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    handles.Joint5Box.String = manualjoint
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
global qlim;


    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(qlim(6,1)) ' and ' num2str(qlim(6,2)) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.Joint6Box.String = '0';
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < qlim(6,1) || qlim(6,2) < manualjoint) 
       uiwait(warndlg(message));
       handles.Joint6Box.String = '0';
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    handles.Joint6Box.String = manualjoint
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
global qlim;


    message = sprintf(['Input must be an number, inclusive between ' ...
            num2str(qlim(7,1)) ' and ' num2str(joint7Max) '. Reset to default value of 0']);
        
    % getting the handle object, converting it to a double and setting it to a
    % temporary board value called manualDieValue.
    manualjoint = str2double(get(hObject,'String'))
    % check if the die value entered is anything other than an integer
    % (when a non-numerical variable gets converted to a double, it becomes
    % nan - not a number. isnan detects this.)
    if isnan(manualjoint)
        uiwait(warndlg(message));
        handles.Joint7Box.String = '0';
        return;
    end
    % check if the joint angle entered is a number outside of the joint
    % limits
    if (manualjoint < qlim(7,1) || joint7Max < manualjoint) 
       uiwait(warndlg(message));
       handles.Joint7Box.String = '0';
       return;
    end
    % setting the global variable for joint angle to the entered joint angle.
    handles.Joint7Box.String = manualjoint
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


% --- Executes during object creation, after setting all properties.
function IRStatus_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IRStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function CollisionStatus_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CollisionStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
