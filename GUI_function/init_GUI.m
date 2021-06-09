function Init_GUI(handles)

% Initialize sliders
set(handles.sld_theta1, 'Max', 180);
set(handles.sld_theta1, 'Min', -180);
set(handles.sld_theta1, 'SliderStep' , [1, 1] / 360 );

set(handles.sld_theta2, 'Max', 180);
set(handles.sld_theta2, 'Min', -180);
set(handles.sld_theta2, 'SliderStep' , [1, 1] / 360 );

set(handles.sld_d3, 'Max', 0.0974);
set(handles.sld_d3, 'Min', -0.0716);
set(handles.sld_theta2, 'SliderStep' , [1, 1] / 2000 );

set(handles.sld_theta4, 'Max', 180);
set(handles.sld_theta4, 'Min', -180);
set(handles.sld_theta4, 'SliderStep' , [1, 1] / 360 );

%text box end-effector init
set(handles.edit_ef_x, 'enable', 'off');
set(handles.edit_ef_y, 'enable', 'off');
set(handles.edit_ef_z, 'enable', 'off');
set(handles.edit_ef_roll, 'enable', 'off');
set(handles.edit_ef_pitch, 'enable', 'off');
set(handles.edit_ef_yaw, 'enable', 'off');

% panels' visibility init 
set(handles.pnl_inverse_kinematics, 'visible','off');
set(handles.pnl_forward_kinematics, 'visible','off');
set(handles.pnl_path_planning, 'visible','off');
set(handles.pnl_trajectory_theta1, 'visible','off');
set(handles.pnl_trajectory_theta2, 'visible','off');
set(handles.pnl_trajectory_d3, 'visible','off');
set(handles.pnl_trajectory_theta4, 'visible','off');
set(handles.pnl_3D_view, 'visible','off');

% axes
resetAxes(handles.axe_3D_view);
resetAxes(handles.axe_q);
resetAxes(handles.axe_q_dot);
resetAxes(handles.axe_q_2dot);


%axes visibility init
handles.axe_theta1.XAxis.Visible = 'off';
handles.axe_theta1.YAxis.Visible = 'off';
handles.axe_theta1_dot.XAxis.Visible = 'off';
handles.axe_theta1_dot.YAxis.Visible = 'off';
handles.axe_theta1_2dot.XAxis.Visible = 'off';
handles.axe_theta1_2dot.YAxis.Visible = 'off';
handles.axe_theta2.XAxis.Visible = 'off';
handles.axe_theta2.YAxis.Visible = 'off';
handles.axe_theta2_dot.XAxis.Visible = 'off';
handles.axe_theta2_dot.YAxis.Visible = 'off';
handles.axe_theta2_2dot.XAxis.Visible = 'off';
handles.axe_theta2_2dot.YAxis.Visible = 'off';
handles.axe_d3.XAxis.Visible = 'off';
handles.axe_d3.YAxis.Visible = 'off';
handles.axe_d3_dot.XAxis.Visible = 'off';
handles.axe_d3_dot.YAxis.Visible = 'off';
handles.axe_d3_2dot.XAxis.Visible = 'off';
handles.axe_d3_2dot.YAxis.Visible = 'off';
handles.axe_theta4.XAxis.Visible = 'off';
handles.axe_theta4.YAxis.Visible = 'off';
handles.axe_theta4_dot.XAxis.Visible = 'off';
handles.axe_theta4_dot.YAxis.Visible = 'off';
handles.axe_theta4_2dot.XAxis.Visible = 'off';
handles.axe_theta4_2dot.YAxis.Visible = 'off';
ylabel(handles.axe_q,'q');
ylabel(handles.axe_q_dot,'q\_dot');
ylabel(handles.axe_q_2dot,'q\_2dot');
end