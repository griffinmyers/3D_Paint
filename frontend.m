% 3D Paint written by William Myers (wgm37) and Guo Jie Chin (gc348) for
% ECE4760, April 2012. This program takes delay information for 3
% ultrasonic transmitter pairs and trilaterates a pen's coordinates in xyz
% space. The result is then displayed in three possible views:
%
% 3D Space: See the drawing from any 3D Projection. This view mode also has
% 'camera' functionality - in camera mode the pen movements will be
% interpretted as a camera position so you can view about the drawn object
%
% Orthogonal Planes: The xz, yz, and xy projections are displayed at the
% same time. 
%
% Stereo: Two 3D images of the drawing are shown with a slighly offset
% angle. The user can wear stereo glasses to see the drawing popout of the
% screen! Camera mode also functions here. 
%
% Additionally, the GUI allows the user to clear the screen, set the 
% background color, select brush color, size and type, and quit. 


function varargout = frontend(varargin)
% Begin initialization code - DO NOT EDIT
% This code simply sets up the matlab GUI for execution and sets up some
% basuic event callbacks. 

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @frontend_OpeningFcn, ...
                   'gui_OutputFcn',  @frontend_OutputFcn, ...
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


% --- Executes just before 3D Paint GUI is made visible.
function frontend_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<INUSL>
% This is the initialization routine for the 3D paint program. Here we set
% up a handful of variables and objects useful for proper operation. Global
% variables are used extensively as a means of passing information from
% callback to call back within the Gui

% Choose default command line output for paint
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global BRUSH_COLOR;                 % Brush Color [%r, %g, %b]
global BRUSH_SIZE;                  % Brush Size [1-10]
global BRUSH_TYPE;                  % Brush Type [-,..,--,.-]
global PEN_MODE;                    % Pen Mode [paint, camera]
global DISPLAY_MODE;                % Display Mode [3D Space, Orthogonal Planes, Stereo]
global INDEX;              
global MEAN_SIZE;                   % Size of mean filtering window
global CAMERA_MEAN_SIZE;            % Size of camera mean filtering window
global MEAN_WINDOW_X;               % The mean window itself
global MEAN_WINDOW_Y;               % The mean window itself
global MEAN_WINDOW_Z;               % The mean window itself
global CAMERA_MEAN_WINDOW_X;        % The camera mean window itself
global CAMERA_MEAN_WINDOW_Y;        % The camera mean window itself
global CAMERA_MEAN_WINDOW_Z;        % The camera mean window itself
global INTERPOLATE_WINDOW_X;        % A size 2 window fot matlab to linearly interpolate data points
global INTERPOLATE_WINDOW_Y;        % A size 2 window fot matlab to linearly interpolate data points
global INTERPOLATE_WINDOW_Z;        % A size 2 window fot matlab to linearly interpolate data points
global INTERPOLATE_WINDOW_X_RIGHT;
global INTERPOLATE_WINDOW_X_LEFT;
global INTERPOLATE_WINDOW_Y_1
global SERIAL;                      % A serial object for communication with the 644
global PUSH_STATE;                  % Whether or not the button on the 644 is pressed
                                    % 1 - No Push; 2 - Maybe Push; 3 - Push;
                                    % 4 - Maybe No Push
global IS_PACKET_GOOD;              % Signal for whether or not we receieved a packet correctly
global LENGTH;                      % Length of RX displacement
global SLEW_OFFSET;
global CURSOR_1;                    % Object for data cursor user feedback
global CURSOR_2;                    % Object for data cursor user feedback
global CURSOR_3;                    % Object for data cursor user feedback
global CURSOR_4;                    % Object for data cursor user feedback

% Initialize some variables
BRUSH_COLOR = [1,.027,0];   % Red
BRUSH_SIZE = 1;             
BRUSH_TYPE = '-';
PEN_MODE = 'paint';
DISPLAY_MODE = '3D Space';
INDEX = 0;
MEAN_SIZE = 3;
CAMERA_MEAN_SIZE = 10;
MEAN_WINDOW_X = zeros(MEAN_SIZE);
MEAN_WINDOW_Y = zeros(MEAN_SIZE);
MEAN_WINDOW_Z = zeros(MEAN_SIZE);
CAMERA_MEAN_WINDOW_X = zeros(CAMERA_MEAN_SIZE);
CAMERA_MEAN_WINDOW_Y = zeros(CAMERA_MEAN_SIZE);
CAMERA_MEAN_WINDOW_Z = zeros(CAMERA_MEAN_SIZE);
INTERPOLATE_WINDOW_X = zeros(2);
INTERPOLATE_WINDOW_Y = zeros(2);
INTERPOLATE_WINDOW_Z = zeros(2);
INTERPOLATE_WINDOW_X_RIGHT = zeros(2);
INTERPOLATE_WINDOW_X_LEFT = zeros(2);
INTERPOLATE_WINDOW_Y_1 = zeros(2);
PUSH_STATE = 1;             % Not pushed
IS_PACKET_GOOD = 0;
LENGTH = .30;             % Displacement between each RX, must be calibrated
SLEW_OFFSET = 3300;         % Distance offset introduced by slew of amps, must be calibrated
AxisSize = 1;               % Size of each axis in meters: [-AxisSize, AxisSize]

% Close out and clean any open serial connections
try %#ok<TRYNC>
    fclose(instrfind);
end

% Initiate a new serial communication object
% Increase the BaudRate here and in uart.c
% Timeout = .1 seconds, this is plently long to wait for an issue
SERIAL = serial('COM7','BaudRate',38500,'DataBits',8,'StopBits',1,'Timeout',.1); 
fopen(SERIAL);

% Check the status on the serial communication link and display results
if(strcmp(get(SERIAL,{'Status'}),'open'))
    % Connected! Set text to 'Open' and green
    set(handles.text1,'String','Open');
    set(handles.text1,'ForegroundColor',[0,1,0]);
else
    % Connected! Set text to 'Closed' and red
    set(handles.text1,'String','Closed');
    set(handles.text1,'ForegroundColor',[1,0,0]);
end

% Turn on the grid for each of the axes
grid(handles.axes1);
grid(handles.axes2);
grid(handles.axes3);
grid(handles.axes4);
grid(handles.axes5);
grid(handles.axes6);

% Init the cursors for each axis, stereo doesn't need cursors
hDC = datacursormode;
set(hDC,'Enable','off');
CURSOR_1 = createDatatip(hDC, handles.axes1);
CURSOR_2 = createDatatip(hDC, handles.axes2);
CURSOR_3 = createDatatip(hDC, handles.axes3);
CURSOR_4 = createDatatip(hDC, handles.axes4);
drawnow

% Set the index bounds on all the axes
axis(handles.axes1,[.2 1.0 -.4 .6 -.4 .6]);
axis(handles.axes2,[.2 1.0 -.4 .6]);
axis(handles.axes3,[.2 1.0 -.4 .6]);
axis(handles.axes4,[-.4 .6 -.4 .6]);
axis(handles.axes5,[.2 1.0 -.4 .6 -.4 .6]);
axis(handles.axes6,[.2 1.0 -.4 .6 -.4 .6]);
axis(handles.axes7,[-AxisSize AxisSize -AxisSize AxisSize]);

% Set the title and color scheme for each plot
title(handles.axes1,'3D Window');
set(handles.axes1,'xcolor',get(gcf,'color'));
set(handles.axes1,'ycolor',get(gcf,'color'));
set(handles.axes1,'zcolor',get(gcf,'color'));

title(handles.axes2,'X-Y Projection');
set(handles.axes2,'xcolor',get(gcf,'color'));
set(handles.axes2,'ycolor',get(gcf,'color'));


title(handles.axes3,'X-Z Projection');
set(handles.axes3,'xcolor',get(gcf,'color'));
set(handles.axes3,'ycolor',get(gcf,'color'));

title(handles.axes4,'Y-Z Projection');
set(handles.axes4,'xcolor',get(gcf,'color'));
set(handles.axes4,'ycolor',get(gcf,'color'));

title(handles.axes5,'Left Eye');
set(handles.axes5,'xcolor',get(gcf,'color'));
set(handles.axes5,'ycolor',get(gcf,'color'));
set(handles.axes5,'zcolor',get(gcf,'color'));
set(handles.axes5,'color',get(gcf,'color'));

title(handles.axes6,'Right Eye');
set(handles.axes6,'xcolor',get(gcf,'color'));
set(handles.axes6,'ycolor',get(gcf,'color'));
set(handles.axes6,'zcolor',get(gcf,'color'));
set(handles.axes6,'color',get(gcf,'color'));

title(handles.axes7,'Red-Blue Stereo');
set(handles.axes7,'xcolor',get(gcf,'color'));
set(handles.axes7,'ycolor',get(gcf,'color'));
set(handles.axes7,'color',get(gcf,'color'));


% Create a timer object used to dispatch sebsequent calls to the 644 for
% more data. If code is running and the timer fires a new instance, it will
% be dopped. The timer fires .05 seconds after the end of the previous
% execution.
handles.guifig = gcf;
handles.tmr = timer('TimerFcn', {@TmrFcn,handles.guifig},'BusyMode','drop',...
    'ExecutionMode','FixedSpacing','Period',.05);

% Store all the handles/gui info
guidata(handles.guifig,handles);
guidata(hObject, handles);
% Start the timer object, we're up and running!
start(handles.tmr);

% Set the 3D camera targets - center in the window
set(handles.axes1, 'CameraTarget', [.6 .1 .1]);

function TmrFcn(src,event,handles) %#ok<INUSL>
% This is the timer function. It is dispatched at regular intervals by the
% timer object set up in the GUI init code. It takes all user input,
% processses and displays it on the 6 axes. 

% Pull down all the necessary global variables for this function
global BRUSH_COLOR;
global BRUSH_SIZE;
global BRUSH_TYPE;
global PEN_MODE;
global DISPLAY_MODE;
global INDEX; 
global MEAN_SIZE;
global CAMERA_MEAN_SIZE;
global MEAN_WINDOW_X;    
global MEAN_WINDOW_Y;      
global MEAN_WINDOW_Z;
global CAMERA_MEAN_WINDOW_X;       
global CAMERA_MEAN_WINDOW_Y;    
global CAMERA_MEAN_WINDOW_Z;    
global INTERPOLATE_WINDOW_X;
global INTERPOLATE_WINDOW_Y;
global INTERPOLATE_WINDOW_Z;
global INTERPOLATE_WINDOW_X_RIGHT;
global INTERPOLATE_WINDOW_X_LEFT;
global INTERPOLATE_WINDOW_Y_1
global SERIAL;
global PUSH_STATE;
global IS_PACKET_GOOD;
global LENGTH;
global SLEW_OFFSET;
global CURSOR_1;
global CURSOR_2;
global CURSOR_3;
global CURSOR_4;


% Request data from the microcontroller. 644 will only reply with data when
% the 'y' character is received. 
fprintf(SERIAL,'y');

handles = guidata(handles);

% Wait for requested data from the microcontroller
% We need PUSH_STATE info for whether or not the button was pressed and the
% delay info
if(SERIAL.BytesAvailable ~= 0)
    data_packet = fscanf(SERIAL,'%d,%d,%d,%d');
    data_packet = data_packet';
end

% Every now and then this code will throw an index out of bounds error-
% surrounded by try/catch to allow for this occasional exception
try
    PUSH_STATE = data_packet(1);
    cycle_delay(1) = data_packet(2) - SLEW_OFFSET;
    cycle_delay(2) = data_packet(3) - SLEW_OFFSET;
    cycle_delay(3) = data_packet(4) - SLEW_OFFSET;
    
    IS_PACKET_GOOD = 1;
catch %#ok<CTCH>
    % In the event of a bad data packet, turn the button off
    PUSH_STATE = 1;
    cycle_delay(1) = 0;
    cycle_delay(2) = 0;
    cycle_delay(3) = 0;
    IS_PACKET_GOOD = 0;
end

if(IS_PACKET_GOOD)
    % Calculate the distance in meters based on the cycle count. 
    % 16000000 is the delay counts per second given by the 644 and 340.29 is
    % the speed of sound.
    distance_point = cycle_delay / 16000000 * 340.29;
    
    if(strcmp(PEN_MODE,'paint')) 
              
        % The median window buffers data values in a sliding window so that matlab
        % can extract the median. Obviously, the window slides with each new data
        % points.
        % MEDIAN_WINDOW = [distance_point, MEDIAN_WINDOW(1:MEDIAN_SIZE-1)];
        % median_filtered_point = median(MEDIAN_WINDOW);
        % The mean window provides the exact same sliding window function as the
        % median window except it implements and average. 
        
        MEAN_WINDOW_X = [distance_point(1), MEAN_WINDOW_X(1:MEAN_SIZE-1)];
        mean_filtered_point(1) = mean(MEAN_WINDOW_X);
        MEAN_WINDOW_Y = [distance_point(2), MEAN_WINDOW_Y(1:MEAN_SIZE-1)];
        mean_filtered_point(2) = mean(MEAN_WINDOW_Y);
        MEAN_WINDOW_Z = [distance_point(3), MEAN_WINDOW_Z(1:MEAN_SIZE-1)];
        mean_filtered_point(3) = mean(MEAN_WINDOW_Z);
        
        % Trilaterate the position in x,y,z space. X point must take the real
        % part, as imperfect data can make the result of the square root
        % imaginary. The real part well approximates where the pen probably
        % is. 
        z_point = (mean_filtered_point(1)^2 - mean_filtered_point(3)^2 + LENGTH^2)/(2*LENGTH);
        y_point = (mean_filtered_point(1)^2 - mean_filtered_point(2)^2 + LENGTH^2)/(2*LENGTH);
        x_point = real(sqrt(mean_filtered_point(1)^2 - y_point^2 - z_point^2));
    
     
        [x_right, x_left, y1] = stereo_plot(z_point,y_point,x_point);
        
        % The interpolate window stores the two most recent data points so that
        % matlab will linearly interpolate based on the two. This is actually what
        % is plotted. 
        INTERPOLATE_WINDOW_X = [INTERPOLATE_WINDOW_X(2),x_point];
        INTERPOLATE_WINDOW_Y = [INTERPOLATE_WINDOW_Y(2),y_point];
        INTERPOLATE_WINDOW_Z = [INTERPOLATE_WINDOW_Z(2),z_point];
        INTERPOLATE_WINDOW_X_RIGHT = [INTERPOLATE_WINDOW_X_RIGHT(2),x_right];
        INTERPOLATE_WINDOW_X_LEFT = [INTERPOLATE_WINDOW_X_LEFT(2),x_left];
        INTERPOLATE_WINDOW_Y_1 = [INTERPOLATE_WINDOW_Y_1(2),y1];

        % Useful time plot for debuggin and calibrating. With this turned
        % on you should be able to move in a plane and see minimal coupling
        % between the three orthogonal components
        %INTERPOLATE_WINDOW_X = [INTERPOLATE_WINDOW_X(2),mean_filtered_point(1)];
        %INTERPOLATE_WINDOW_Y = [INTERPOLATE_WINDOW_Y(2),mean_filtered_point(2)];
        %INTERPOLATE_WINDOW_Z = [INTERPOLATE_WINDOW_Z(2),mean_filtered_point(3)];

        % Update all of the cursor positions. The cursors are updated irrespective
        % of whether or not the paint button is pressed. This is separated up into
        % cases a) because it improves runtime efficiency (the set() command takes
        % a suprisingly long time to execute) and b) because setting a cursor in an
        % axes that is not displayed will throw errors
        switch(DISPLAY_MODE)
            case '3D Space'
                set(CURSOR_1,'Position',[INTERPOLATE_WINDOW_X(1) INTERPOLATE_WINDOW_Y(1) INTERPOLATE_WINDOW_Z(1)],'String','');
            case 'Orthogonal Planes'
                set(CURSOR_2,'Position',[INTERPOLATE_WINDOW_X(1) INTERPOLATE_WINDOW_Y(1)],'String','');
                set(CURSOR_3,'Position',[INTERPOLATE_WINDOW_X(1) INTERPOLATE_WINDOW_Z(1)],'String','');
                set(CURSOR_4,'Position',[INTERPOLATE_WINDOW_Y(1) INTERPOLATE_WINDOW_Z(1)],'String','');
        end

        % If the button is pushed, go ahead and plot some points!
        if(PUSH_STATE ~= 1) 
            % Uncomment these two lines for the calibration mentioned
            % above. Comment out the normal plots. 
            %plot(handles.axes2,INTERPOLATE_WINDOW_X,[INDEX-1 INDEX],INTERPOLATE_WINDOW_Y,[INDEX-1 INDEX],INTERPOLATE_WINDOW_Z,[INDEX-1 INDEX]);
            %INDEX = INDEX + 1;
            
            plot3(handles.axes1,INTERPOLATE_WINDOW_X,INTERPOLATE_WINDOW_Y,INTERPOLATE_WINDOW_Z,'LineWidth',BRUSH_SIZE,'Color',BRUSH_COLOR,'LineStyle',BRUSH_TYPE);
            plot(handles.axes2,INTERPOLATE_WINDOW_X,INTERPOLATE_WINDOW_Y,'LineWidth',BRUSH_SIZE,'Color',BRUSH_COLOR,'LineStyle',BRUSH_TYPE);
            plot(handles.axes3,INTERPOLATE_WINDOW_X,INTERPOLATE_WINDOW_Z,'LineWidth',BRUSH_SIZE,'Color',BRUSH_COLOR,'LineStyle',BRUSH_TYPE);
            plot(handles.axes4,INTERPOLATE_WINDOW_Y,INTERPOLATE_WINDOW_Z,'LineWidth',BRUSH_SIZE,'Color',BRUSH_COLOR,'LineStyle',BRUSH_TYPE);
            plot3(handles.axes5,INTERPOLATE_WINDOW_X,INTERPOLATE_WINDOW_Y,INTERPOLATE_WINDOW_Z,'LineWidth',BRUSH_SIZE,'Color',BRUSH_COLOR,'LineStyle',BRUSH_TYPE);
            plot3(handles.axes6,INTERPOLATE_WINDOW_X,INTERPOLATE_WINDOW_Y,INTERPOLATE_WINDOW_Z,'LineWidth',BRUSH_SIZE,'Color',BRUSH_COLOR,'LineStyle',BRUSH_TYPE);
            plot(handles.axes7,INTERPOLATE_WINDOW_X_LEFT,INTERPOLATE_WINDOW_Y_1,'Color', [1 0 0]);
            plot(handles.axes7,INTERPOLATE_WINDOW_X_RIGHT,INTERPOLATE_WINDOW_Y_1, 'Color', [0 0 1]);
        end

        % Plotting will change the Visibility parameter of a plot, fix that. 
        set_axes(DISPLAY_MODE, handles)

        % This basically makes the axes objects global so that the data can be
        % consecutively placed on multiple function calls
        hold(handles.axes1,'on');
        hold(handles.axes2,'on');
        hold(handles.axes3,'on');
        hold(handles.axes4,'on');
        hold(handles.axes5,'on');
        hold(handles.axes6,'on');
        hold(handles.axes7,'on');

        % Used for calibration. Under normal operation INDEX is not
        % incremented so the plot will not be cleared unless the user does
        % it via the Clear button. 
        if(INDEX == 100)
           cla(handles.axes1);
           cla(handles.axes2);
           cla(handles.axes3);
           cla(handles.axes4);
           cla(handles.axes5);
           cla(handles.axes6);
           cla(handles.axes7);
           INDEX = 0;
        end
     else
         % Camera Mode - this mode uses a longer mean filter to smooth out
         % all jitter - response isn't as important as it is with drawing!
         
        CAMERA_MEAN_WINDOW_X = [distance_point(1), CAMERA_MEAN_WINDOW_X(1:CAMERA_MEAN_SIZE-1)];
        camera_mean_filtered_point(1) = mean(CAMERA_MEAN_WINDOW_X);
        CAMERA_MEAN_WINDOW_Y = [distance_point(2), CAMERA_MEAN_WINDOW_Y(1:CAMERA_MEAN_SIZE-1)];
        camera_mean_filtered_point(2) = mean(CAMERA_MEAN_WINDOW_Y);
        CAMERA_MEAN_WINDOW_Z = [distance_point(3), CAMERA_MEAN_WINDOW_Z(1:CAMERA_MEAN_SIZE-1)];
        camera_mean_filtered_point(3) = mean(CAMERA_MEAN_WINDOW_Z);    
        
        % Trilaterate the position in x,y,z space. X point must take the real
        % part, as imperfect data can make the result of the square root
        % imaginary. The real part well approximates where the pen probably
        % is. 
        camera_z_point = (camera_mean_filtered_point(1)^2 - camera_mean_filtered_point(3)^2 + LENGTH^2)/(2*LENGTH);
        camera_y_point = (camera_mean_filtered_point(1)^2 - camera_mean_filtered_point(2)^2 + LENGTH^2)/(2*LENGTH);
        camera_x_point = real(sqrt(camera_mean_filtered_point(1)^2 - camera_y_point^2 - camera_z_point^2));
         

        switch(DISPLAY_MODE)
            case '3D Space'
                set(handles.axes1, 'CameraPosition', 3*[camera_x_point camera_y_point camera_z_point]);
            case 'Stereo'
                % Take the view angle for the right eye and calculate the
                % position of the left eye for stereo imaging. Algorithm
                % graciously provided by Bruce Land. 
                set(handles.axes5, 'CameraPosition', 3*[camera_x_point camera_y_point camera_z_point]);
                from = 3*[camera_x_point camera_y_point camera_z_point];
                to = get(handles.axes5,'cameratarget');
                up = get(handles.axes5,'cameraupvector');
                d=.1; % Eye spacing
                % Get the position of the left eye
                lefteye=from+d*cross((from-to),up)/sqrt(sum((from-to).^2)) ;
                set(handles.axes6,'cameraposition',lefteye);
        end
        
     end
end

guidata(handles.guifig, handles);

function[] = set_axes(str, handles)
% This helper function takes and input value str as the desired view mode
% and input object handles to toggle the correct axes. It toggles the
% display mode for a given axes and all children of the axes object (i.e.
% data). 

% Get an handle to all children objects of the 6 axes (i.e. data)
h1 = get(handles.axes1,'Children');
h2 = get(handles.axes2,'Children');
h3 = get(handles.axes3,'Children');
h4 = get(handles.axes4,'Children');
h5 = get(handles.axes5,'Children');
h6 = get(handles.axes6,'Children');
h7 = get(handles.axes7,'Children');

switch str
    case '3D Space'
        
        % Turn off visibility on axes and children
        set(h2,'Visible','off');
        set(h3,'Visible','off');
        set(h4,'Visible','off');
        set(h5,'Visible','off');
        set(h6,'Visible','off');
        set(h7,'Visible','off');
        set(handles.axes2,'Visible','off');
        set(handles.axes3,'Visible','off');
        set(handles.axes4,'Visible','off');
        set(handles.axes5,'Visible','off');
        set(handles.axes6,'Visible','off');
        set(handles.axes7,'Visible','off');
        
        % Matlab complains if you turn something on that is already on, so
        % make sure it is not!
        if(strcmp(get(handles.axes1,'Visible'), 'off'))
            % Turn axes and children visibility on
            set(handles.axes1,'Visible','on');
            set(h1,'Visible','on');
        end
        
    case 'Orthogonal Planes'
        
        % Turn off visibility on axes and children
        set(h1,'Visible','off');
        set(h5,'Visible','off');
        set(h6,'Visible','off');
        set(h7,'Visible','off');
        set(handles.axes1,'Visible','off');
        set(handles.axes5,'Visible','off');
        set(handles.axes6,'Visible','off');
        set(handles.axes7,'Visible','off');
        
        % Matlab complains if you turn something on that is already on, so
        % make sure it is not!
        if(strcmp(get(handles.axes2,'Visible'),'off'))
            % Turn axes and children visibility on
            set(handles.axes2,'Visible','on');
            set(h2,'Visible','on');
        end
         if(strcmp(get(handles.axes3,'Visible'),'off'))
             % Turn axes and children visibility on
            set(handles.axes3,'Visible','on');
            set(h3,'Visible','on');
        end
        if(strcmp(get(handles.axes4,'Visible'),'off'))
            % Turn axes and children visibility on
            set(handles.axes4,'Visible','on');
            set(h4,'Visible','on');
        end
        
    case 'Stereo'
        % Turn off visibility on axes and children
        set(h1,'Visible','off');
        set(h2,'Visible','off');
        set(h3,'Visible','off');
        set(h4,'Visible','off');
        set(handles.axes1,'Visible','off');
        set(handles.axes2,'Visible','off');
        set(handles.axes3,'Visible','off');
        set(handles.axes4,'Visible','off');
 
        % Matlab complains if you turn something on that is already on, so
        % make sure it is not!
        if(strcmp(get(handles.axes5,'Visible'),'off'))
            % Turn axes and children visibility on
            set(handles.axes5,'Visible','on');
            set(h5,'Visible','on');
        end
        if(strcmp(get(handles.axes6,'Visible'),'off'))
            % Turn axes and children visibility on
            set(handles.axes6,'Visible','on');
            set(h6,'Visible','on');
        end
        if(strcmp(get(handles.axes7,'Visible'),'off'))
            % Turn axes and children visibility on
            set(handles.axes7,'Visible','on');
            set(h7,'Visible','on');
        end
       
end

% --- Outputs from this function are returned to the command line.
function varargout = frontend_OutputFcn(hObject, eventdata, handles)  %#ok<INUSL>
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
% This function selects the brush size to be used by the program. Brush
% size is selected in the GUI via a dropdown menu. The function simply
% switches on the selected value and assigned a value in [1,10] to a global
% variable to be used elsewhere in the program. 

global BRUSH_SIZE;

% Get the selected dropdown value
contents = cellstr(get(hObject,'String'));
str = contents{get(hObject,'Value')};

switch(str)
    case '1 Pixels'
        BRUSH_SIZE = 1;
    case '2 Pixels'
        BRUSH_SIZE = 2;
    case '3 Pixels'
        BRUSH_SIZE = 3;
    case '4 Pixels'
        BRUSH_SIZE = 4;
    case '5 Pixels'
        BRUSH_SIZE = 5;
    case '6 Pixels'
        BRUSH_SIZE = 6;
    case '7 Pixels'
        BRUSH_SIZE = 7;
    case '8 Pixels'
        BRUSH_SIZE = 8;
    case '9 Pixels'
        BRUSH_SIZE = 9;
    case '10 Pixels'
        BRUSH_SIZE = 10;
end

% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
% This function changes the brush type used by the program. The GUI
% selector for this is a drop down menu with the 4 possible brush types.
% The function simply switches on the input and assigns the correct plot
% syntax to a global variable to be used elsewhere in the program. 

global BRUSH_TYPE;

% Get the value of the dropdown selected
contents = cellstr(get(hObject,'String'));
str = contents{get(hObject,'Value')};

switch(str)
    case 'Solid Line'
        BRUSH_TYPE = '-';
    case 'Dashed Line'
        BRUSH_TYPE = '--';
    case 'Dotted Line'
        BRUSH_TYPE = ':';
    case 'Dash-Dot Line'
        BRUSH_TYPE = '-.';
end


% --- Executes when selected object is changed in uipanel9.
function uipanel9_SelectionChangeFcn(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
% This function changes the display mode used by the program. The
% input argument can be one of three values: '3D Space', 'Stereo', and 
% 'Orthogonal Planes'. The axes are toggled via a helper function
% set_axes()
global DISPLAY_MODE;

str = get(eventdata.NewValue,'String');
DISPLAY_MODE = str;
set_axes(str, handles);


% --- Executes when selected object is changed in uipanel10.
function uipanel10_SelectionChangeFcn(hObject, eventdata, handles) %#ok<DEFNU,INUSD,INUSL>
% This function sets the brush color currently in use. There is a 
% 12 button toggle array on the GUI, each with it's display color
% signaling the desired brush color. The function stores this background
% color in a global variable for use elsewhere in the program. 

global BRUSH_COLOR;

BRUSH_COLOR = get(eventdata.NewValue,'BackGroundColor');
    


% --- Executes when selected object is changed in uipanel6.
function uipanel6_SelectionChangeFcn(hObject, eventdata, handles) %#ok<INUSD,INUSL,DEFNU>
% This function selects what mode the Pen should be acting in.
% We get the input argument from the display name of the toggle
% button on the GUI and switch accordingly. Result is stored in
% a global variable for use elsewhere in the program.

global PEN_MODE;
global CURSOR_1;
global CURSOR_2;
global CURSOR_3;
global CURSOR_4;

str = get(eventdata.NewValue,'String');

switch(str)
    case 'Paint'
        PEN_MODE = 'paint';
        set(CURSOR_1,'Visible','on');
        set(CURSOR_2,'Visible','on');
        set(CURSOR_3,'Visible','on');
        set(CURSOR_4,'Visible','on');
    case 'Camera'
        PEN_MODE = 'camera';
        set(CURSOR_1,'Visible','off');
        set(CURSOR_2,'Visible','off');
        set(CURSOR_3,'Visible','off');
        set(CURSOR_4,'Visible','off');
    
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
% This function clears all plots to create a new canvas
% No input arguments, simply runs when the 'Clear Screen' button
% is pressed. 

% cla() clears the specified axes
cla(handles.axes1);
cla(handles.axes2);
cla(handles.axes3);
cla(handles.axes4);
cla(handles.axes5);
cla(handles.axes6);
cla(handles.axes7);


% --- Executes when selected object is changed in uipanel12.
function uipanel12_SelectionChangeFcn(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
% This function will toggle the background color for all the plots
% Only two options from the toggle button are 'Black' and 'White'

% Get the pressed Button's display name
str = get(eventdata.NewValue,'String');

switch(str)
    case 'Black'
        % Set all axes background accordingly
        set(handles.axes1,'Color','black');
        set(handles.axes2,'Color','black');
        set(handles.axes3,'Color','black');
        set(handles.axes4,'Color','black');
        set(handles.axes1,'xcolor', [0.4,0.4,0.4]);
        set(handles.axes1,'ycolor', [0.4,0.4,0.4]);
        set(handles.axes1,'zcolor', [0.4,0.4,0.4]);
        set(handles.axes2,'xcolor', [0.4,0.4,0.4]);
        set(handles.axes2,'ycolor', [0.4,0.4,0.4]);
        set(handles.axes2,'zcolor', [0.4,0.4,0.4]);
        set(handles.axes3,'xcolor', [0.4,0.4,0.4]);
        set(handles.axes3,'ycolor', [0.4,0.4,0.4]);
        set(handles.axes3,'zcolor', [0.4,0.4,0.4]);
        set(handles.axes4,'xcolor', [0.4,0.4,0.4]);
        set(handles.axes4,'ycolor', [0.4,0.4,0.4]);
        set(handles.axes4,'zcolor', [0.4,0.4,0.4]);
    case 'White'
        % Set all axes background accordingly
        set(handles.axes1,'Color','white');
        set(handles.axes2,'Color','white');
        set(handles.axes3,'Color','white');
        set(handles.axes4,'Color','white');
        set(handles.axes1,'xcolor', get(gcf,'color'));
        set(handles.axes1,'ycolor', get(gcf,'color'));
        set(handles.axes1,'zcolor', get(gcf,'color'));
        set(handles.axes2,'xcolor', get(gcf,'color'));
        set(handles.axes2,'ycolor', get(gcf,'color'));
        set(handles.axes2,'zcolor', get(gcf,'color'));
        set(handles.axes3,'xcolor', get(gcf,'color'));
        set(handles.axes3,'ycolor', get(gcf,'color'));
        set(handles.axes3,'zcolor', get(gcf,'color'));
        set(handles.axes4,'xcolor', get(gcf,'color'));
        set(handles.axes4,'ycolor', get(gcf,'color'));
        set(handles.axes4,'zcolor', get(gcf,'color'));
end

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
% This function stops the timer object and clears everything out
% necessary for proper termination. It runs when the 'Exit' button
% is pressed

stop(handles.tmr);
delete(handles.tmr);
delete(handles.guifig);
clear all
close all


% --------------------------------------------------------------------
function file_Callback(hObject, eventdata, handles)


% --------------------------------------------------------------------
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global DISPLAY_MODE;

switch(DISPLAY_MODE)
    case '3D Space'
        f = getframe(handles.axes1);
        imwrite(f.cdata,strcat('3D Space-',date,'.jpg'),'jpg'); 
    case 'Orthogonal Planes'
        f = getframe(handles.axes2);
        imwrite(f.cdata,strcat('XY Projection-',date,'.jpg'),'jpg'); 
        f = getframe(handles.axes3);
        imwrite(f.cdata,strcat('XZ Projection-',date,'.jpg'),'jpg'); 
        f = getframe(handles.axes4);
        imwrite(f.cdata,strcat('YZ Projection-',date,'.jpg'),'jpg'); 
    case 'Stereo'
        f = getframe(handles.axes5);
        imwrite(f.cdata,strcat('Left Eye-',date,'.jpg'),'jpg'); 
        f = getframe(handles.axes6);
        imwrite(f.cdata,strcat('Right Eye-',date,'.jpg'),'jpg'); 
        f = getframe(handles.axes7);
        imwrite(f.cdata,strcat('Red Blue-',date,'.jpg'),'jpg'); 
end

function [x_right,x_left,y1]=stereo_plot(x,y,z)
% The function calculate coordinates for the left-eye and right-eye images
% of 3D graphics.
% the input are x-y-z coordinates, the output are coordinates of two plots
% (the y-axis is shared for both eyes).
% the inputs should be in units of centimeters, if not - the scaling will
% changed.
% the z-axis is the depth of the image, positive z is out of the screen,
% negative z is into the screen
%
% programmed by Moshe Lindner, April 2010 (c).

% initial parameters
d=.20; %distance from screen to eyes [cm]
i_l=-.0032; % left eye x-position [cm]
i_r=.0032; % right eye x-position [cm]

% calculations
x_right=i_r - d.*(x-i_r)./(z-d);
x_left=i_l - d.*(x-i_l)./(z-d);
y1=- d.*(y)./(z-d);
