 function varargout = SCARA_robot_GUI(varargin)
% Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @SCARA_robot_GUI_OpeningFcn, ...
                       'gui_OutputFcn',  @SCARA_robot_GUI_OutputFcn, ...
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


% --- Executes just before SCARA_robot_GUI is made visible.
function SCARA_robot_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
    addpath(genpath(pwd));
    handles.color_array = ['m','c','r','b','k'];
    handles.pre_ef_p = zeros(3,1);
    handles.mov_cycle = 0; handles.mov_s_ = 0; handles.mov_q = 0; handles.mov_q_dot = 0; handles.mov_q_2dot = 0;handles.mov_qr = 0;
    handles.timer = timer;
    handles.timer.TimerFcn = {@update_GUI, hObject};
    handles.timer.Period = 0.007;
    handles.timer.ExecutionMode = 'fixedRate';
    init_GUI(handles);
    handles.path_planning_mode = 1;
    DH_params = [0,0,0,0;0.325000000000000,0,0.00540000000000000,0;0.275000000000000,0,0,1.57079632679490;0,0,-0.0716000000000000,0;0,3.14159265358979,-0.00400000000000000,0];
    handles.robot = C_Scara(DH_params,0.1);
    handles.robot.initialize();  
    handles.Ts = 0.1;
    handles.output = hObject; % Choose default command line output for SCARA_robot_GUI
    guidata(hObject, handles); % Update handles structure

% --- Outputs from this function are returned to the command line.
function varargout = SCARA_robot_GUI_OutputFcn(hObject, eventdata, handles) 

    varargout{1} = handles.output; % Get default command line output from handles structure

function btn_startSim_Callback(hObject, eventdata, handles)
    if get(hObject,'String') == "Start Sim"
        if handles.robot.get('model_status') ~= "running"
            sim(handles.robot.get('model'));
        elseif handles.robot.get('model_status') == "running"
            handles.robot.startRobot();
            start(handles.timer);
            set(hObject,'String',"Stop Sim")
        end
    elseif get(hObject,'String') == "Stop Sim"
        handles.robot.stopTimer();
        stop(handles.timer);
        set_param(handles.robot.get('model'), 'SimulationCommand', 'stop');
        pause(1);
        set(hObject,'String',"Start Sim")
    end
    guidata(hObject, handles); % Update handles structure
    
function btn_DynamicsSim_Callback(hObject, eventdata, handles)
    value = get(hObject,'Value');
    if value == 1
        handles.robot.useDynamics();
    elseif value == 0
        handles.robot.stopDynamics();
    end
    guidata(hObject, handles); % Update handles structure
    
function btn_exit_Callback(hObject, eventdata, handles)
    guidata(hObject, handles); % Update handles structure
    
function btn_reset_Callback(hObject, eventdata, handles)
    % reset axes
    cla(handles.axe_3D_view,'reset');
    grid(handles.axe_3D_view,'on');
    hold(handles.axe_3D_view,'on');
    
    guidata(hObject, handles); % Update handles structure
    
function sld_theta1_Callback(hObject, eventdata, handles)
    value = d2r(get(hObject,'Value'));
    q = handles.robot.get('q');
    q(1) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function sld_theta2_Callback(hObject, eventdata, handles)
    value = d2r(get(hObject,'Value'));
    q = handles.robot.get('q');
    q(2) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function sld_d3_Callback(hObject, eventdata, handles)
    value = get(hObject,'Value');
    q = handles.robot.get('q');
    q(3) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function sld_theta4_Callback(hObject, eventdata, handles)
    value = d2r(get(hObject,'Value'));
    q = handles.robot.get('q');
    q(4) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function edit_theta1_Callback(hObject, eventdata, handles)
    value = d2r(str2double(get(hObject,'String')));
    q = handles.robot.get('q');
    q(1) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function edit_theta2_Callback(hObject, eventdata, handles)
    value = d2r(str2double(get(hObject,'String')));
    q = handles.robot.get('q');
    q(2) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function edit_d3_Callback(hObject, eventdata, handles)
    value = str2double(get(hObject,'String'));
    q = handles.robot.get('q');
    q(3) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function edit_theta4_Callback(hObject, eventdata, handles)
    value = d2r(str2double(get(hObject,'String')));
    q = handles.robot.get('q');
    q(4) = value;
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function btn_forward_Callback(hObject, eventdata, handles)
    q = zeros(4,1);
    q(1) = d2r(str2double(get(handles.edit_forward_theta1,'String')));
    q(2) = d2r(str2double(get(handles.edit_forward_theta2,'String')));
    q(3) = str2double(get(handles.edit_forward_d3,'String'));
    q(4) = d2r(str2double(get(handles.edit_forward_theta4,'String')));
    handles.robot.forwardKinematics(q);
    guidata(hObject, handles); % Update handles structure

function pop_problem_Callback(hObject, eventdata, handles)
    set(handles.pnl_forward_kinematics, 'visible','off');
    set(handles.pnl_inverse_kinematics, 'visible','off');
    set(handles.pnl_path_planning, 'visible','off');
    set(handles.sld_theta1, 'visible','off'); set(handles.sld_theta2, 'visible','off'); set(handles.sld_d3, 'visible','off'); set(handles.sld_theta4, 'visible','off');
    pop_value = get(hObject,'Value');
    switch pop_value
        case 1
            q = handles.robot.get('q');
            set(handles.sld_theta1, 'visible','on'); set(handles.sld_theta2, 'visible','on'); set(handles.sld_d3, 'visible','on'); set(handles.sld_theta4, 'visible','on');
            set(handles.sld_theta1,'Value',q(1)); set(handles.sld_theta2,'Value',q(2)); set(handles.sld_d3,'Value',q(3)); set(handles.sld_theta4,'Value',q(4));
        case 2
            set(handles.pnl_forward_kinematics, 'visible','on');
        case 3
            set(handles.pnl_inverse_kinematics, 'visible','on');
        case 4
            set(handles.pnl_path_planning, 'visible','on');
        otherwise
    end

function btn_inverse_Callback(hObject, eventdata, handles)
    ef_disired_pose = zeros(6,1);
    ef_disired_pose(1) = str2double(get(handles.edit_inverse_x,'String'));
    ef_disired_pose(2) = str2double(get(handles.edit_inverse_y,'String'));
    ef_disired_pose(3) = str2double(get(handles.edit_inverse_z,'String'));
    ef_disired_pose(5) = pi;
    ef_disired_pose(6) = d2r(str2double(get(handles.edit_inverse_yaw,'String')));
    handles.robot.inverseKinematics(ef_disired_pose);
    guidata(hObject, handles); % Update handles structure

function btn_planning_Callback(hObject, eventdata, handles)
    handles.color = handles.color_array(randi(length(handles.color_array)));
    
    resetAllAxes(handles);
    ef_desired_p = zeros(3,1);
    ef_desired_p(1) = str2double(get(handles.edit_motion_x,'String')); ef_desired_p(2) = str2double(get(handles.edit_motion_y,'String')); ef_desired_p(3) = str2double(get(handles.edit_motion_z,'String'));
    ef_desired_p2 = zeros(3,1);
    ef_desired_p2(1) = str2double(get(handles.edit_motion_x2,'String')); ef_desired_p2(2) = str2double(get(handles.edit_motion_y2,'String')); ef_desired_p2(3) = str2double(get(handles.edit_motion_z2,'String'));
    switch handles.path_planning_mode
        case 1
            handles.robot.movLinear(ef_desired_p);
        case 2
            handles.robot.movCircular2D(ef_desired_p, ef_desired_p2);
        otherwise
            warning("Path planning");
    end
    pause(1);
    mov_cycle = handles.robot.get('mov_cycle');
    handles.mov_cycle = zeros(1,mov_cycle);
    for i = 1:mov_cycle
        handles.mov_cycle(i) = i;
    end
    handles.mov_s_ = zeros(3,mov_cycle); handles.mov_q = zeros(4,mov_cycle); handles.mov_qr = zeros(4,mov_cycle); 
    handles.mov_q_dot = zeros(4,mov_cycle); handles.mov_q_2dot = zeros(4,mov_cycle);
    guidata(hObject, handles); % Update handles structure
    
function btn_path_planning_run_Callback(hObject, eventdata, handles)
    handles.robot.mov();
    guidata(hObject, handles); % Update handles structure

function pop_strategy_Callback(hObject, eventdata, handles)
    value = get(hObject,'Value');
    handles.path_planning_mode = value;
    if value == 2
        set(handles.edit_motion_x2, 'visible','on');
        set(handles.edit_motion_y2, 'visible','on');
        set(handles.edit_motion_z2, 'visible','on');
        set(handles.edit_motion_yaw2, 'visible','on');
        set(handles.text_motion_x2, 'visible','on');
        set(handles.text_motion_y2, 'visible','on');
        set(handles.text_motion_z2, 'visible','on');
        set(handles.text_motion_yaw2, 'visible','on');
    else
        set(handles.edit_motion_x2, 'visible','off');
        set(handles.edit_motion_y2, 'visible','off');
        set(handles.edit_motion_z2, 'visible','off');
        set(handles.edit_motion_yaw2, 'visible','off');
        set(handles.text_motion_x2, 'visible','off');
        set(handles.text_motion_y2, 'visible','off');
        set(handles.text_motion_z2, 'visible','off');
        set(handles.text_motion_yaw2, 'visible','off');    
    end
    guidata(hObject, handles); % Update handles structure

function edit_v_max_Callback(hObject, eventdata, handles)
    handles.robot.set('per_v',str2double(get(hObject,'String')));
    guidata(hObject, handles); % Update handles structure
    
function edit_a_max_Callback(hObject, eventdata, handles)
    handles.robot.set('a_max',str2double(get(hObject,'String')));
    guidata(hObject, handles); % Update handles structure

function pop_trajectory_Callback(hObject, eventdata, handles)
    set(handles.pnl_trajectory_tool_space, 'visible','off');
    set(handles.pnl_trajectory_theta1, 'visible','off');
    set(handles.pnl_trajectory_theta2, 'visible','off');
    set(handles.pnl_trajectory_d3, 'visible','off');
    set(handles.pnl_trajectory_theta4, 'visible','off');
    set(handles.pnl_3D_view, 'visible','off');
    pop_value = get(hObject,'Value');
    switch pop_value
        case 1
            set(handles.pnl_trajectory_tool_space, 'visible','on');
        case 2
            set(handles.pnl_trajectory_theta1, 'visible','on');
            set(handles.pnl_trajectory_theta2, 'visible','on');
            set(handles.pnl_trajectory_d3, 'visible','on');
            set(handles.pnl_trajectory_theta4, 'visible','on');
        case 3
            set(handles.pnl_3D_view, 'visible','on');
        otherwise
    end

function edit_motion_y2_Callback(hObject, eventdata, handles)

function edit_motion_x2_Callback(hObject, eventdata, handles)

function edit_motion_z2_Callback(hObject, eventdata, handles)

function edit_motion_yaw2_Callback(hObject, eventdata, handles)

function sld_theta1_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function sld_theta2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function sld_d3_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function sld_theta4_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function edit_theta1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_theta2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_d3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_theta4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_forward_theta1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_forward_theta2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_forward_d3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_forward_theta4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_ef_x_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_ef_y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_ef_z_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_ef_roll_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_ef_pitch_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_ef_yaw_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function pop_problem_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_inverse_yaw_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_inverse_z_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_inverse_y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_inverse_x_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_x_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_z_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_yaw_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_v_max_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function pop_strategy_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_a_max_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function pop_trajectory_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_y2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_x2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_z2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_motion_yaw2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_forward_theta1_Callback(hObject, eventdata, handles)
function edit_forward_theta2_Callback(hObject, eventdata, handles)
function edit_forward_d3_Callback(hObject, ~, handles)
function edit_forward_theta4_Callback(hObject, eventdata, handles)
function edit_inverse_yaw_Callback(hObject, eventdata, handles)
function edit_inverse_z_Callback(hObject, eventdata, handles)
function edit_inverse_y_Callback(hObject, eventdata, handles)
function edit_inverse_x_Callback(hObject, eventdata, handles)
function edit_motion_y_Callback(hObject, eventdata, handles)
function edit_motion_x_Callback(hObject, eventdata, handles)
function edit_motion_z_Callback(hObject, eventdata, handles)
function edit_motion_yaw_Callback(hObject, eventdata, handles)
function edit_ef_x_Callback(hObject, eventdata, handles)
function edit_ef_y_Callback(hObject, eventdata, handles)
function edit_ef_z_Callback(hObject, eventdata, handles)
function edit_ef_roll_Callback(hObject, eventdata, handles)
function edit_ef_pitch_Callback(hObject, eventdata, handles)
function edit_ef_yaw_Callback(hObject, eventdata, handles)

function update_GUI(hObject, eventdata, hFigure)
    handles = guidata(hFigure);
    cycle = handles.robot.get('cycle');
    % Joint variables
    qr = handles.robot.get('qr');
    q = handles.robot.get('q');
    q_dot = handles.robot.get('q_dot');
    q_2dot = handles.robot.get('q_2dot');
    set(handles.edit_theta1,'String',num2str(r2d(q(1))));
    set(handles.edit_theta2,'String',num2str(r2d(q(2))));
    set(handles.edit_d3,'String',num2str(q(3)));
    set(handles.edit_theta4,'String',num2str(r2d(q(4))));
    
    % End-effector
    ef_pose = handles.robot.get('ef_pose');
    s_ = handles.robot.get('s_');
    set(handles.edit_ef_x, 'String',num2str(ef_pose(1)));
    set(handles.edit_ef_y, 'String',num2str(ef_pose(2)));
    set(handles.edit_ef_z, 'String',num2str(ef_pose(3)));
    set(handles.edit_ef_roll, 'String',num2str(r2d(ef_pose(4))));
    set(handles.edit_ef_pitch, 'String',num2str(r2d(ef_pose(5))));
    set(handles.edit_ef_yaw, 'String',num2str(r2d(ef_pose(6))));
    if handles.robot.get('isMoving') == true
        handles.mov_s_(:,cycle) = s_; handles.mov_q(:,cycle) = q; 
        handles.mov_q_dot(:,cycle) = q_dot; handles.mov_q_2dot(:,cycle) = q_2dot;
        if handles.robot.get('isUsingDynamics') == true 
            handles.mov_qr(:,cycle) = qr;
        end
        if cycle == handles.mov_cycle(length(handles.mov_cycle))
            plot(handles.axe_q,handles.mov_cycle*handles.Ts,handles.mov_s_(1,:),handles.color,'LineWidth',2);
            plot(handles.axe_q_dot,handles.mov_cycle*handles.Ts,handles.mov_s_(2,:),handles.color,'LineWidth',2);
            plot(handles.axe_q_2dot,handles.mov_cycle*handles.Ts,handles.mov_s_(3,:),handles.color,'LineWidth',2);
            plot(handles.axe_theta1,handles.mov_cycle*handles.Ts,handles.mov_q(1,:),'r','LineWidth',1);
            plot(handles.axe_theta2,handles.mov_cycle*handles.Ts,handles.mov_q(2,:),'r','LineWidth',1);
            plot(handles.axe_d3,handles.mov_cycle*handles.Ts,handles.mov_q(3,:),'r','LineWidth',1);
            plot(handles.axe_theta4,handles.mov_cycle*handles.Ts,handles.mov_q(4,:),'r','LineWidth',1);
            if handles.robot.get('isUsingDynamics') == true
                plot(handles.axe_theta1,handles.mov_cycle*handles.Ts,handles.mov_qr(1,:),'b','LineWidth',1);
                plot(handles.axe_theta2,handles.mov_cycle*handles.Ts,handles.mov_qr(2,:),'b','LineWidth',1);
                plot(handles.axe_d3,handles.mov_cycle*handles.Ts,handles.mov_qr(3,:),'b','LineWidth',1);
                plot(handles.axe_theta4,handles.mov_cycle*handles.Ts,handles.mov_qr(4,:),'b','LineWidth',1);
            else
                plot(handles.axe_theta1_dot,handles.mov_cycle*handles.Ts,handles.mov_q_dot(1,:),handles.color,'LineWidth',1);
                plot(handles.axe_theta2_dot,handles.mov_cycle*handles.Ts,handles.mov_q_dot(2,:),handles.color,'LineWidth',1);
                plot(handles.axe_d3_dot,handles.mov_cycle*handles.Ts,handles.mov_q_dot(3,:),handles.color,'LineWidth',1);
                plot(handles.axe_theta4_dot,handles.mov_cycle*handles.Ts,handles.mov_q_dot(4,:),handles.color,'LineWidth',1);
                plot(handles.axe_theta1_2dot,handles.mov_cycle*handles.Ts,handles.mov_q_2dot(1,:),handles.color,'LineWidth',1);
                plot(handles.axe_theta2_2dot,handles.mov_cycle*handles.Ts,handles.mov_q_2dot(2,:),handles.color,'LineWidth',1);
                plot(handles.axe_d3_2dot,handles.mov_cycle*handles.Ts,handles.mov_q_2dot(3,:),handles.color,'LineWidth',1);
                plot(handles.axe_theta4_2dot,handles.mov_cycle*handles.Ts,handles.mov_q_2dot(4,:),handles.color,'LineWidth',1);
            end
        end
            plot3(handles.axe_3D_view, [handles.pre_ef_p(1) ef_pose(1)], [handles.pre_ef_p(2) ef_pose(2)], [handles.pre_ef_p(3) ef_pose(3)],handles.color,'LineWidth',3);
    end
    handles.pre_ef_p = ef_pose(1:3);
    guidata(hFigure, handles); % Update handles structure

function figure1_CloseRequestFcn(hObject, eventdata, handles)
    handles.robot.delete();
    stop(handles.timer);
    delete(hObject);
