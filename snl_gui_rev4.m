function varargout = snl_gui(varargin)
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

% Last Modified by GUIDE v2.5 14-May-2018 11:38:28

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
    global boardValueMin;
    global boardValueMax;
    global dieValueMin;
    global dieValueMax;
    
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
global robot;
global realistic;
global fps;
global scale;
global steps;
global s;
global qZero;
global eStop;
    moveValue = dieValue;
    % This is where the trajectory planning should occur.
    
    % This is where the trajectory operation should occur.
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
    end

    % --- Executes on button press in MoveByPosition.
function MoveByPosition_Callback(hObject, eventdata, handles)
% hObject    handle to MoveByPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global boardValue
global robot;
global realistic;
global fps;
global scale;
global steps;
global s;
global qZero;
global eStop;
    moveValue = boardValue;
    % This is where the trajectory planning should occur.
    
    % This is where the trajectory operation should occur.
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
