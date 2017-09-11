%% Script to derive a CLF-QP controller 
clear all; close all; clc;


%% Load system dynamic model
controllerName = 'IO_PD';
modelName = 'ballbot2D';
addpath('../../models')
load(strcat('syms_model_',modelName,'.mat'));

% Model Parameters values
[params, unpacked_params] = get_ballbot2D_model_params();
load(unpacked_params);

%% Define output function
syms phi_d dphi_d real
h = phi - phi_d;
dh = dphi - dphi_d;

%% Subsitute params values into dynamics
f_num = subs(f);
g_num = subs(g);

% Compute Lie derivatives
Lgh = jacobian(h,x)*g_num;

Lfh = jacobian(h,x)*f;
LgLfh = jacobian(Lfh,x)*g;

Lf2h  = jacobian(Lfh,x)*f;

%% I/O w/ PD linear system: 
% deta = F*eta + G*u
eta = [h; dh];
F = [0 1; 0 0]; G = [0; 1];

% Using I/O linearization with PD control, 
% determine PD gains using LQR
Q = diag([500 100]); R = eye(size(u));

K = lqr(F,G,Q,R);

%% I/O w/ Min-norm system
% Construct a Lyapunov function
A = [0 1; -K(1) -K(2)];
syms h dh c3 epsilon real
Q = eye(size(eta,1));
%P = lyap(A,Q);

P = care(F,G,Q);
P_eps = [1/epsilon 0 ; 0 1]*P*[1/epsilon 0 ; 0 1];

V_eps = eta'*P*eta;

LfVeps = eta'*(F'*P_eps + P_eps*F)*eta;
LgVeps = 2*eta'*P_eps*G;

psi0 = LfVeps + c3/epsilon*V_eps;
psi1 = LgVeps;

%% Auto-generate efficient matlab function to lie derivatives
FolderName = 'autogen_fun';
if ~exist(FolderName,'dir')
    warning('directory does not exist:%s\n It will be created.', FolderName)
    mkdir(FolderName);
end

matlabFunction(LgLfh,Lf2h, 'File',fullfile(FolderName,strcat('autofun_LieDerv_',modelName,'_',controllerName)),'Vars',vertcat(q,dq,'phi_d','dphi_d'));
matlabFunction(psi0,psi1, 'File',fullfile(FolderName,strcat('autofun_psi_',modelName,'_',controllerName)),'Vars',vertcat(q,dq,'phi_d','dphi_d','c3','epsilon'));
matlabFunction(V_eps,LfVeps,LgVeps, 'File',fullfile(FolderName,strcat('autofun_CLFLieDerv_',modelName,'_',controllerName)),'Vars',vertcat(q,dq,'phi_d','dphi_d','epsilon'));
