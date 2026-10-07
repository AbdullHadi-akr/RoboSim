%==============================================
%
%   ROBOTIK
%   Übung 1.2
%   Planarer 2-DOF Manipulator in Simulink
%
%----------------------------------------------
%
%   Prof. Dr.-Ing. Frank Bender
%   Elektrotechnik und Informationstechnik
%   DHBW Stuttgart
%
%----------------------------------------------
%
%   03.08.2026 - Initiale Erstellung der Aufgabe
%
%==============================================

close all
clear
clc

%% Parameter für 2-DOF Manipulator in Simulink

manip.l1 = 3.0;            % [m] Link 1 length
manip.l2 = 1.0;            % [m] Link 2 length
manip.m1 = 30.0;           % [kg] Link 1 point mass
manip.m2 = 10.0;           % [kg] Link 2 point mass

%--------- TODO Teilaufgabe f) --------------------------------------------
manip.m1plant = manip.m1;%*1.05;  % Simulierter Modellfehler
manip.m2plant = manip.m2;%*0.95;  % Simulierter Modellfehler
%--------------------------------------------------------------

% Slender-link cross section used for geometry and inertia approximation
manip.linkWidth = 0.05;    % [m]
manip.linkThickness = 0.05;% [m]

% Joint limits (from sketch) in deg (matches Revolute Joint limit units)
manip.theta1_min = -60;
manip.theta1_max = 110;
manip.theta2_min = -120;
manip.theta2_max = 120;

% Limit contact properties (units set by joint mask: deg-based)
manip.kLimit = 1e4;        % [N*m/deg]
manip.dLimit = 100;        % [N*m/(deg/s)]
manip.limitTransition = 1; % [deg]

% Gravity in world frame KS0 (y-axis points upward in the sketch)
manip.g = 9.81;            % [m/s^2]

% Densities so the brick solids realize the requested masses
manip.rho1 = manip.m1/(manip.l1*manip.linkWidth*manip.linkThickness);
manip.rho2 = manip.m2/(manip.l2*manip.linkWidth*manip.linkThickness);

% Position references and trajectory feedforward terms
manip.q1_ref = deg2rad(40);   % [rad]
manip.q2_ref = deg2rad(20);   % [rad]
manip.q1d_ref = 0;            % [rad/s]
manip.q2d_ref = 0;            % [rad/s]
manip.q1dd_ref = 0;           % [Nm] optional additive torque bias channel
manip.q2dd_ref = 0;           % [Nm] optional additive torque bias channel

% Joint-space PI(+D) torque feedback gains
manip.kp1 = 260;              % [N*m/rad]
manip.kd1 = 90;               % [N*m*s/rad]
manip.kp2 = 75;               % [N*m/rad]
manip.kd2 = 28;               % [N*m*s/rad]

% Actuator saturation
manip.tau1_max = 4500;        % [N*m]
manip.tau2_max = 1800;        % [N*m]


