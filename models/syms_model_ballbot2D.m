 % FILENAME: syms_model_ballbot2D.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 8/26/2017
%
% DESCRIPTION:
% This script will derive the equation of motion using the 
% Lagrian Dynamics method of a 2D Ballbot model. Coordinate scheme based on
% [3]
% 
% The 3 different notations for the equation of motions are derived as in 
%
%   
%   (1) D(q)ddq + C(q,dq)dq + G(q) = B(q)u + J(q)'*F_external
%
%       D(q) = Mass matrix
%       C(q,dq) = Coriolis forces
%       G(q) = gravitational forces
%       B(q) = output matrix
%       J(q) = contact jacobian
% Refer to Apendix B.4.5 in [1] for derivation.
% 
%   (2) D(q)*dqq + H(q,dq) = B*u + J(q)'*F_external
%   (3) Control affine representation: 
%       dx = f(x) + g(x)*u
%       x = [q;dq]
%
% Generalized coordinate set
%   q = [x y phi alpha l];
%   u = actuator forces
%   lambda = contact forces
%
% Notation: 
%   x = x-position of body
%   y = y-position of body
%   phi = body orientation
%   alpha = hip joint angle
%   l = leg length
%
% References:
% [1] Robot Dynamics and Control by Spong and Vidyasagar (1989), page 142, Eq. (6.3.12)
% [2] Westervelt, Eric R., et al. Feedback control of dynamic 
%     bipedal robot locomotion. Vol. 28. CRC press, 2007.
% [3] Nagarajan, Umashankar, George Kantor, and Ralph L. Hollis. 
%     "Trajectory planning and control of an underactuated dynamically stable single 
%     spherical wheeled mobile robot." Robotics and Automation, 2009. ICRA'09. IEEE 
%     International Conference on. IEEE, 2009.

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clear all; close all; clc;

%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------
% Define model's name
modelName = 'ballbot2D';

% Define symbolic variables
syms phi theta dphi dtheta ddphi ddtheta real % State variables
syms M_body M_ball I_body I_ball grav L r real % Model parameters
syms Lf Lfy real
syms tau real % Inputs

params_vec = [M_body M_ball I_body I_ball L r grav];

% Configuration coordintes and velocities
% theta and phi are in radias, positive counter-clockwise
% State vectors
q = [theta; phi];
dq = [dtheta; dphi];
ddq = [ddtheta; ddphi];

% Input Vector
u = tau;

%% ----------------------------------------------------------
%   SYSTEM KINEMATICS
% -----------------------------------------------------------
% Ball position and velocity
p_ball = [r*(theta + phi); 0];
v_ball = jacobian(p_ball,q)*dq;

% Body CoM position and velocity 
p_body = [L*sin(phi); L*cos(phi)] + p_ball;
v_body = jacobian(p_body,q)*dq;

%% ----------------------------------------------------------
%   SYSTEM DYNAMICS
% -----------------------------------------------------------
% Kinetic Energy
KE_ball = 0.5*M_ball*v_ball'*v_ball + 0.5*I_ball*(dtheta + dphi)^2;
KE_body = 0.5*M_body*v_body'*v_body + 0.5*I_body*(dphi)^2;
KE = simplify(KE_ball + KE_body);

% Potential Energy
PE = M_body*grav*p_body(2);
PE = simplify(PE);

% Lagrangian
Lag = KE - PE;

% Vector of actuated state variables
gamma_vec = [ theta]; 

% D(q)*ddq + C(q)*dq + G(q) = B*u
[D_mtx,C_mtx,G_vec,B_mtx] = EulerLagrangeEoM_method1(KE,PE,q,dq, gamma_vec);

% Lagrangian equations of motion of the form
% D(q)*ddq + H(q,dq) = B*u + J'*F
[ D, H, B ] = EulerLagrangeEoM_method2( Lag, q, dq, ddq, gamma_vec);
ddq = D\(B*u - H);

% ODE Form (control affine state space model)
% dx = f(x) + g(x)*u
[ x, f, g ] = EoM2CntrlAffine( D, H, B, q, dq );
dx = f+g*u;

%% ---------------------------------------------------------
%   CLOSED LOOP W/ PID DYAMICS
% -----------------------------------------------------------
syms Theta real
syms theta_r dtheta_r itheta_r real
syms Kp Ki Kd real

fbar = [ theta; dtheta; dphi; -D\H];

gbar = [ 0; 0; 0; D\(B*(-Kp*L*(theta-theta_r) - Kd*(dtheta - dtheta_r) - Ki*(Theta - itheta_r)))];

%% ----------------------------------------------------------
%   SAVE MODEL
% -----------------------------------------------------------
%save(strcat('syms_model_',modelName,'.mat'));

%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Auto-generate efficient matlab function to compute dynamics
FolderName = 'autogen_fun';
if ~exist(FolderName,'dir')
    warning('directory does not exist:%s\n It will be created.', FolderName)
    mkdir(FolderName);
end

matlabFunction(dx, 'File',fullfile('autogen_fun',strcat('autofun_dx_ode_',modelName)),'Vars',horzcat(q',dq',u',params_vec));
matlabFunction(ddq, 'File',fullfile('autogen_fun',strcat('autofun_ddq_lagr_',modelName)),'Vars',horzcat(q',dq',u',params_vec));
matlabFunction(p_ball,p_body, 'File',fullfile('autogen_fun',strcat('autofun_kinematics_',modelName)),'Vars',horzcat(q',dq',u',params_vec));
