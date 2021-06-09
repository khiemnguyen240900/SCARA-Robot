Topic: SCARA Robot Simulation using MATLAB Simscape, GUIDE and Simulink.
--------------DO NOT DELETE THIS--------------
Author: Nguyen Gia Khiem
Department of Control Engineering and Automation
Ho Chi Minh City University of Technology
Ho Chi Minh City, Vietnam
Email: khiemiist@gmail.com || khiem.nguyen2409@hcmut.edu.vn
--------------DO NOT DELETE THIS--------------
MATLAB Object Oriented Programming with classes:
- Class Joint
- Class Link
- Class Frame
- Class Robot (in development)
- Class SCARA
How to use:
- Run add_path.m
- Run SCARA_robot_GUI.m and wait until everything is loaded
- Click "Start Sim" button and wait until the model is loaded
- Click "Start Sim" button to load robot parameters
- There are 4 options about problem to choose in the dropbox at the bottom corner of the GUI
	+ Forward_1: Using Slider to change the joint variables
	+ Forward_2: Solving forward kinematics problem
	+ Inverse: Solving inverse kinematics problem
	+ Path & Trajectory planning: Solving path & trajectory planning problem
- Click "Dynamics" button to enable the motor and PID controller simulation, re-click to disabl
- In "Path & Trajectory planning": 
	There are 2 options of path:
	+ "Linear": only require 1 destination point
	+ "Circular 2D": require 1 intermediate point and 1 destination point
	"per_v" edit box: (value 0.0-1.0) set the maximum velocity of the end-effector - v_max = per_v*sqrt(s_max*a_max)
	"a_max" edit box: (default: 0.05) set the maximum acceleration of the end-effector
- There are 3 options about axes to choose in the dropbox at the top of the GUI
	+ End-effector 3D view
	+ Joint variables view
	+ Tool space view
- Click "Stop Sim" to stop simulation and "Exit" button to exit the GUI

