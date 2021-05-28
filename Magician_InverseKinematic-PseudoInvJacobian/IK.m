function varargout = GUI_Magician(varargin)
% IK MATLAB code for IK.fig
%      IK, by itself, creates a new IK or raises the existing
%      singleton*.
%
%      H = IK returns the handle to a new IK or the handle to
%      the existing singleton*.
%
%      IK('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IK.M with the given input arguments.
%
%      IK('Property','Value',...) creates a new IK or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IK_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IK_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help IK

% Last Modified by GUIDE v2.5 14-Jan-2021 15:50:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IK_OpeningFcn, ...
                   'gui_OutputFcn',  @IK_OutputFcn, ...
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



% --- Executes just before IK is made visible.
function IK_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IK (see VARARGIN)

% Choose default command line output for IK
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IK wait for user response (see UIRESUME)
% uiwait(handles.figure1);

delete(instrfind({'Port'}, {'COM7'}))
clear a;
global a;
a = arduino('COM7');

dem=0;
assignin('base','dem',dem); 

jinit=[90;118;-157];
assignin('base','jinit',jinit); 
assignin('base','jstart',jinit); 
axes(handles.axes1);
FKdraw(jinit(1,1),jinit(2,1),jinit(3,1));
ax_properties = get(gca);
assignin('base','pov',ax_properties.View);
FK=evalin('base','FK');
pos_start=[FK(1,16);FK(2,16);FK(3,16)];
pos_target=[FK(1,16);FK(2,16);FK(3,16)];
assignin('base','pos_start',pos_start);
assignin('base','pos_target',pos_target);



% --- Outputs from this function are returned to the command line.
function varargout = IK_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Reset.
function Reset_Callback(hObject, eventdata, handles)
global a;
axes(handles.axes1);
jinit=evalin('base','jinit');
assignin('base','jstart',jinit); 
FKdraw(jinit(1,1),jinit(2,1),jinit(3,1));
FK=evalin('base','FK');

set( handles.pos_x,'String', num2str(2.4));
set( handles.pos_y,'String', num2str(176.2265));
set( handles.pos_z,'String', num2str(139.1823));

set( handles.Theta_1,'String', num2str(90));
set( handles.Theta_2,'String', num2str(118));
set( handles.Theta_3,'String', num2str(-157));

assignin('base','movement',0);

robot_magican(a,round(jinit(1,1)),round(jinit(2,1)),-round(jinit(3,1)),3);



%% --- Executes on button press in inversebutton.
function inversebutton_Callback(hObject, eventdata, handles)
global a;

dem=evalin('base','dem');
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;

pos_start=evalin('base','pos_start');

PX = str2double(handles.pos_x.String);
PY = str2double(handles.pos_y.String);
PZ = str2double(handles.pos_z.String);

tocdo = str2double(handles.tocdo.String);

pos_target=[PX;PY;PZ];
assignin('base','pos_target',pos_target); 

%pos_target=evalin('base','pos_target');
jstart=evalin('base','jstart');
j1=jstart(1,1);
j2=jstart(2,1);
j3=jstart(3,1);
if pos_target==pos_start
   errordlg('Set target position first','No Movement'); 
   movement=0;
   assignin('base','movement',movement);
else
   movement=1;
   assignin('base','movement',movement);
   delta=divelo(pos_start,pos_target);
end

while evalin('base','movement')==1
FK=evalin('base','FK');
Jac=Jacobian(FK);
Jacinv=pinv(Jac);
dXYZ=delta(1:3,1)/10;
dTheta=Jacinv*dXYZ;
dTheta1=radtodeg(dTheta(1,1));
dTheta2=radtodeg(dTheta(2,1));
dTheta3=radtodeg(dTheta(3,1));
j1=j1+dTheta1;
j2=j2+dTheta2;
j3=j3+dTheta3;

axes(handles.axes1);
FKdraw(j1,j2,j3)

pos_new=[FK(1,16);FK(2,16);FK(3,16)];
delta=divelo(pos_new,pos_target);
EucError=delta(5,1)^2;
OrinError=delta(6,1)^2;
set( handles.textEucE,'String', num2str(EucError,3));

handles.Theta_1.String = num2str(j1);
handles.Theta_2.String = num2str(j2);
handles.Theta_3.String = num2str(j3);

hold off
dem = dem+0.1;
axes(handles.axes2);
plot(dem,j1,'-o','LineWidth',1);
axis([0 dem 0 180])
grid on;
hold on
axes(handles.axes3);
plot(dem,j2,'-o','LineWidth',1);
axis([0 dem 0 120])
grid on;
hold on
axes(handles.axes4);
plot(dem,j3,'-o','LineWidth',1);
axis([0 dem -180 0])
grid on;
hold on

if EucError <10^-3
   movement=0; 
   assignin('base','movement',movement);
   assignin('base','pos_start',pos_new);
   jstart=[j1;j2;j3];
   assignin('base','jstart',jstart);
   handles.Theta_1.String = num2str(j1);
   handles.Theta_2.String = num2str(j2);
   handles.Theta_3.String = num2str(j3);
   robot_magican(a,round(j1),round(j2),-round(j3),tocdo);
   %assignin('base','dem',dem);
end 

% if all(delta(:) <= 0.1)
%    movement=0; 
%    assignin('base','movement',movement);
%    assignin('base','pos_start',pos_new);
%    assignin('base','jstart',jstart);
% end 

    
%dEucXY=delta(4,1);
%dEucXYZ=delta(5,1);

assignin('base','theta',jstart);

end


function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double


% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double


% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double


% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_x_Callback(hObject, eventdata, handles)
% hObject    handle to pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_x as text
%        str2double(get(hObject,'String')) returns contents of pos_x as a double


% --- Executes during object creation, after setting all properties.
function pos_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_y_Callback(hObject, eventdata, handles)
% hObject    handle to pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_y as text
%        str2double(get(hObject,'String')) returns contents of pos_y as a double


% --- Executes during object creation, after setting all properties.
function pos_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_z_Callback(hObject, eventdata, handles)
% hObject    handle to pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_z as text
%        str2double(get(hObject,'String')) returns contents of pos_z as a double


% --- Executes during object creation, after setting all properties.
function pos_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in forwardbutton.
function forwardbutton_Callback(hObject, eventdata, handles)
% hObject    handle to forwardbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a;
dem = evalin('base','dem');
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;

Th_1 = str2double(handles.Theta_1.String);
Th_2 = str2double(handles.Theta_2.String);
Th_3 = str2double(handles.Theta_3.String);
tocdo = str2double(handles.tocdo.String);

pos_start=evalin('base','pos_start');

Th_4=360-(Th_2+Th_3);
j=[Th_1 Th_2 Th_3 Th_4;103.14 0 0 2.4;30.6 177.5 190 73.65;90 0 0 0];
FK=DHkine(j);
Q=XYZkine(FK);
pos_target=[Q(:,5)];

jstart=evalin('base','jstart');
j1=jstart(1,1);
j2=jstart(2,1);
j3=jstart(3,1);
if pos_target==pos_start
   errordlg('Set target position first','No Movement'); 
   movement=0;
   assignin('base','movement',movement);
else
   movement=1;
   assignin('base','movement',movement);
   delta=divelo(pos_start,pos_target);
end

while evalin('base','movement')==1
FK=evalin('base','FK');
Jac=Jacobian(FK);
Jacinv=pinv(Jac);
dXYZ=delta(1:3,1)/10;  
dTheta=Jacinv*dXYZ;
dTheta1=radtodeg(dTheta(1,1));
dTheta2=radtodeg(dTheta(2,1));
dTheta3=radtodeg(dTheta(3,1));
j1=j1+dTheta1;
j2=j2+dTheta2;
j3=j3+dTheta3;

axes(handles.axes1);
FKdraw(j1,j2,j3)

pos_new=[FK(1,16);FK(2,16);FK(3,16)];
delta=divelo(pos_new,pos_target);
EucError=delta(5,1)^2;
OrinError=delta(6,1)^2;
set( handles.textEucE,'String', num2str(EucError,3));

handles.pos_x.String = num2str(pos_new(1));
handles.pos_y.String = num2str(pos_new(2));
handles.pos_z.String = num2str(pos_new(3));

hold off
dem = dem+0.1;
axes(handles.axes2);
plot(dem,j1,'-o','LineWidth',1);
axis([0 dem 0 180])
grid on;
hold on
axes(handles.axes3);
plot(dem,j2,'-o','LineWidth',1);
axis([0 dem 0 120])
grid on;
hold on
axes(handles.axes4);
plot(dem,j3,'-o','LineWidth',1);
axis([0 dem -180 0])
grid on;
hold on

if EucError <10^-2
   movement=0; 
   assignin('base','movement',movement);
   assignin('base','pos_start',pos_new);
   jstart=[j1;j2;j3];
   assignin('base','jstart',jstart);
   handles.pos_x.String = num2str(pos_new(1));
   handles.pos_y.String = num2str(pos_new(2));
   handles.pos_z.String = num2str(pos_new(3));
   robot_magican(a,round(j1),round(j2),-round(j3),tocdo);
   %assignin('base','dem',dem);
end 

if all(delta(:) <= 0.1)
   movement=0; 
   assignin('base','movement',movement);
   assignin('base','pos_start',pos_new);
   assignin('base','jstart',jstart);
end 

    
%dEucXY=delta(4,1);
%dEucXYZ=delta(5,1);

assignin('base','theta',jstart);

end


% --- Executes on button press in textEucE.
function textEucE_Callback(hObject, eventdata, handles)
% hObject    handle to textEucE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of textEucE


% --- Executes during object creation, after setting all properties.
function textEucE_CreateFcn(hObject, eventdata, handles)
% hObject    handle to textEucE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function tocdo_Callback(hObject, eventdata, handles)
% hObject    handle to tocdo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tocdo as text
%        str2double(get(hObject,'String')) returns contents of tocdo as a double


% --- Executes during object creation, after setting all properties.
function tocdo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tocdo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
