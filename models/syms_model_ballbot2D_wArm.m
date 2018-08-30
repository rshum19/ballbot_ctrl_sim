clear; clc;
%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------
syms phi theta  alpha dphi dtheta dalpha ddphi ddtheta ddalpha real
syms M_body M_ball M_arm I_body I_ball I_arm grav L r L_armjoint l_arm real
syms tau tau_a real

%% Name system model
modelName = 'planarBB_wArm';

% State vectors: Configuration coordintes and velocities
% theta and phi are in radias, positive counter-clockwise
q = [theta; phi; alpha];
dq = [dtheta; dphi; dalpha];
ddq = [ddtheta; ddphi; ddalpha];
u = [tau; tau_a];

params_vec = [M_body M_ball M_arm I_body I_ball I_arm L r L_armjoint l_arm grav];

%% ----------------------------------------------------------
%   SYSTEM KINEMATICS
% -----------------------------------------------------------
derivative = @(f)(jacobian(f,q)*dq); % Chain rule

% Define state equilibirum point
x_eq = [zeros(size(q)); zeros(size(dq))];

% Ball position and velocity
p_ball = [r*(theta + phi); 0];
v_ball2 = derivative(p_ball);
v_ball = jacobian(p_ball,q)*dq;

% Body CoM position and velocity 
p_body = [L*sin(phi); L*cos(phi)] + p_ball;
v_body = jacobian(p_body,q)*dq;

% Arm CoM position and velocity
p_arm = p_ball + [L_armjoint*sin(phi) - l_arm*sin(phi - alpha); L_armjoint*cos(phi) - l_arm*cos(phi - alpha)];
v_arm = jacobian(p_arm,q)*dq;

% System CoM in cartesian coordinates
p_com = (p_body*M_body + p_arm*M_arm)./(M_body + M_arm);
v_com = jacobian(p_com,q)*dq;

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------
% Kinetic Energy
KE_ball = 0.5*M_ball*v_ball'*v_ball + 0.5*I_ball*(dtheta + dphi)^2;
KE_body = 0.5*M_body*v_body'*v_body + 0.5*I_body*(dphi)^2;
KE_arm = 0.5*M_arm*v_arm'*v_arm + 0.5*I_arm*(dphi-dalpha)^2;
KE = simplify(KE_ball + KE_body + KE_arm);

% Potential Energy
PE_body = M_body*grav*p_body(2);
PE_arm = M_arm*grav*p_arm(2);
PE = simplify(PE_body + PE_arm);

% Lagrangian
Lag = KE - PE;

% Vector of controlled/actuated state variables
gamma_vec = [theta, alpha];

% D(q)*ddq + C(q)*dq + G(q) = B*u
[D_mtx,C_mtx,G_vec,B_mtx] = EulerLagrangeEoM_method1(KE,PE,q,dq, gamma_vec);

% Lagrangian equations of motion of the form
% D(q)*dqq + H(q,dq) = B*u + J'*F
[ D, H, B ] = EulerLagrangeEoM_method2( Lag, q, dq, ddq, gamma_vec);
ddq = D\(B*u - H);

% ODE Form (affine state space model)
% dx = f(x) + g(x)*u
x = [q;dq];
f = [dq; -D\H];
g = [zeros(size(B)); D\B];

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

Foldername = 'autogen_fun';
if ~exist(FolderName,'dir')
    warning('directory does not exist:%s\n It will be created.', FolderName)
    mkdir(FolderName);
end

matlabFunction(f+g*u, 'File',fullfile('autogen_fun',strcat('autofun_ddq_ode_',modelName)),'Vars',horzcat(q',dq',u',params_vec));
matlabFunction(D\(B*u - H), 'File',fullfile('autogen_fun',strcat('autofun_ddq_lagr_',modelName)),'Vars',horzcat(q',dq',u',params_vec));

% Generate Kinematic Functions
% -----------------------------

