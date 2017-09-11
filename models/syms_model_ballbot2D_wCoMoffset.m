%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------
% Define symbolic variables
syms phi theta beta dphi dtheta ddphi ddtheta real
syms M_body M_ball I_body I_ball grav L r real
syms Lf Lfy real
syms tau real

% Name system model
modelName = 'planarBB_wCoMoffset';

% Configuration coordintes and velocities
% theta and phi are in radias, positive counter-clockwise
q = [theta; phi];
dq = [dtheta; dphi];
ddq = [ddtheta; ddphi];
u = tau;

params_vec = [M_body M_ball I_body I_ball L r beta grav];

%% ----------------------------------------------------------
%   SYSTEM KINEMATICS
% -----------------------------------------------------------
% Ball position and velocity
p_ball = [r*(theta + phi); 0];
v_ball = jacobian(p_ball,q)*dq;

% Body CoM position and velocity 
p_body = p_ball + [-L*sin(phi + beta); L*cos(phi + beta)];
v_body = jacobian(p_body,q)*dq;

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------
% Kinetic Energy
KE_ball = 0.5*M_ball*v_ball'*v_ball + 0.5*I_ball*(dtheta + dphi)^2;
KE_body = 0.5*M_body*v_body'*v_body + 0.5*I_body*(dphi)^2;
KE = simplify(KE_ball + KE_body);

% Potential Energy
PE = M_body*grav*p_body(2);

% Lagrangian
Lag = KE - PE;

% Lagrangian equations of motion of the form
% D(q)*dqq + H(q,dq) = B*u + J'*F
LHS = jacobian(jacobian(Lag, dq)', [q;dq])*[dq;ddq]  -  jacobian(Lag, q)' ;
D = jacobian(LHS, ddq) ;
H = simplify(LHS - D*ddq) ;
B = [1; 0 ];

% ODE Form (affine state space model)
% dx = f(x) + g(x)*u
x = [q;dq];
f = [dq; -D\H];
g = [zeros(size(dq)); D\B];

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
