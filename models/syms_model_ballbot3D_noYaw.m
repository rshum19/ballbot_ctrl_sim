% Need to remove the yaw
% [1] Spong, Mark W., and M. Vidyasagar. Robot Dynamics and Control. Wiley, 2004.
clear all; close all; clc;
%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------
syms alpha beta gamma dalpha dbeta dgamma  ddgamma ddbeta ddalpha real
syms theta_x theta_y dtheta_x dtheta_y ddtheta_x ddtheta_y real
syms M_body M_ball grav L r real
syms Ixx_body Iyy_body Izz_body Ixx_ball Iyy_ball Izz_ball real
syms tau_x tau_y real
syms I_ball real
I_body = diag([Ixx_body,Iyy_body,Izz_body]);
%I_ball = diag([Ixx_ball,Iyy_ball,Izz_ball]);

% Name system model
modelName = 'ballbot3D_noYaw';

% Configuration coordintes and velocities
% theta and phi are in radias, positive counter-clockwise
q = [theta_x; theta_y; gamma; beta];
dq = [dtheta_x; dtheta_y; dgamma; dbeta];
ddq = [ddtheta_x; ddtheta_y; ddgamma; ddbeta];
u = [tau_x; tau_y];

%params_vec = [M_body M_ball Ixx_body Iyy_body Izz_body Ixx_ball Iyy_ball Izz_ball L r grav];
params_vec = [M_body M_ball Ixx_body Iyy_body Izz_body I_ball L r grav];

alpha = 0;
dalpha = 0;
%% ----------------------------------------------------------
%   SYSTEM KINEMATICS
% -----------------------------------------------------------
Rx = [  1   0   0; 
        0 cos(gamma) -sin(gamma); 
        0 sin(gamma) cos(gamma)];

Ry = [  cos(beta) 0   sin(beta);
        0   1   0;
        -sin(beta)  0   cos(beta)];

Rz = [  cos(alpha)  -sin(alpha) 0;
        sin(alpha)  cos(alpha)  0;
        0   0   1];

e_i = [1; 0; 0];
e_j = [0; 1; 0];
e_k = [0; 0; 1];
V_body = [dgamma; dbeta];

% Net rotation
R = Rz * Ry * Rx;

% Linear position and velocity of ball w.r.t. Frame {0}
P_ball = r * [  theta_x;    theta_y;    0];
V_ball = r * [  dtheta_x;    dtheta_y;   0];
w_ball = [  dtheta_x;   dtheta_y;   0];

% Body angular velocity w.r.t Frame {1}
w_15 = [   0;  0;  dalpha] + Rz*[  0;  dbeta;  0] + Rz*Ry*[    dgamma; 0;  0];
%w_15 = [   0;  0;  dalpha] + Rz*[  dgamma;  0;  0] + Rz*Rx*[    0; dbeta;  0];

% Linear Position and Velocity of CoM w.r.t. to Frame {4} at center of ball rotate from
% Frame {1}
P_CoM_2 = [0; 0; L];
V_CoM_2 = [0; 0; 0];    % CoM does not move up and down body (i.e. L = constant)

% Linear Position and Velocity of CoM w.r.t. to Frame {0}
P_CoM_0 = P_ball + R*P_CoM_2;

S_15 = [   0,  -w_15(3),  w_15(2);...
           w_15(3),   0,  -w_15(1);...
           -w_15(2),  w_15(1),   0];
       
V_CoM_0 = V_ball + S_15*R*P_CoM_2 + R*V_CoM_2;

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------
% Kinetic Energy
KE_ball = 0.5*M_ball*V_ball'*V_ball + 0.5*w_ball'*I_ball*w_ball;
KE_body = 0.5*M_body*V_CoM_0'*V_CoM_0 + 0.5*w_15'*R*I_body*R'*w_15;
KE = simplify(KE_ball + KE_body );

% Potential Energy
PE_body = M_body*grav*P_CoM_0(3);
PE = simplify(PE_body);

% Lagrangian
Lag = KE - PE;

% Lagrangian equations of motion of the form
% D(q)*dqq + H(q,dq) = B*u + J'*F
LHS = jacobian(jacobian(Lag, dq)', [q;dq])*[dq;ddq]  -  jacobian(Lag, q)' ;
D = jacobian(LHS, ddq) ;
H = simplify(LHS - D*ddq) ;
B = [1 0; 0 1; 0 0; 0 0];

% ODE Form (affine state space model)
% dx = f(x) + g(x)*u
x = [q;dq];
f = [dq; -D\H];
g = [zeros(size(B)); D\B];

% -----------------------------------------------------------------
%
%  Calculate model matrices
%
% -----------------------------------------------------------------
% D*ddq + C*dq + G = B*u

% gravity vector
G_vect = jacobian(PE,q).';
G_vect = simplify(G_vect);

% mass-inertial matrix
D_mtx = simplify(jacobian(KE,dq).');
D_mtx = simplify(jacobian(D_mtx,dq));

% Coriolis and centrifugal matrix
syms C_mtx real
n=max(size(q));
for k=1:n
	for j=1:n
		C_mtx(k,j)=0*grav;
		for i=1:n
			C_mtx(k,j)=C_mtx(k,j)+1/2*(diff(D_mtx(k,j),q(i)) + ...
				diff(D_mtx(k,i),q(j)) - ...
				diff(D_mtx(i,j),q(k)))*dq(i);
		end
	end
end
C_mtx=simplify(C_mtx);

%% ----------------------------------------------------------
%   SAVE MODEL
% -----------------------------------------------------------
FolderName = 'models';
if ~exist(FolderName,'dir')
    warning('directory does not exist:%s\n It will be created.', FolderName)
    mkdir(FolderName);
end

save(fullfile('models',strcat('syms_model_',modelName,'.mat')));

%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Auto-generate efficient matlab function to compute dynamics
FolderName = 'autogen_fun';
if ~exist(FolderName,'dir')
    warning('directory does not exist:%s\n It will be created.', FolderName)
    mkdir(FolderName);
end

matlabFunction(f+g*u, 'File',fullfile('autogen_fun',strcat('autofun_ddq_ode_',modelName)),'Vars',horzcat(q',dq',u',params_vec));
matlabFunction(D\(B*u - H), 'File',fullfile('autogen_fun',strcat('autofun_ddq_lagr_',modelName)),'Vars',horzcat(q',dq',u',params_vec));
