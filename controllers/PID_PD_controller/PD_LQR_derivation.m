%% Script to derive a PD-LQR controller 
clear all; close all; clc;


%% Load system dynamic model
controllerName = 'IO_PD';
modelName = 'ballbot2D';
%addpath('../../models')
load(strcat('syms_model_',modelName,'.mat'));

% Model Parameters values
[params, unpacked_params] = get_ballbot2D_model_params();
load(unpacked_params);

%% Define output function
syms phi_d dphi_d real
h = phi - phi_d;
dh = dphi - dphi_d;

%% Linearize System
Alin = jacobian(dx,x);
Blin = jacobian(dx,u);

%% Subsitute params values into linear dynamics
phi = 0; dphi = 0;
Alin_num = double(subs(Alin));
Blin_num = double(subs(Blin));


%% Calculate LQR Gains
Q = [100, 0, 0, 0;
         0, 0, 0, 0;
         0, 0, 1000, 0;
         0, 0, 0, 0]; R = eye(size(u));
     
Klqr = lqr(Alin_num,Blin_num,Q,R);

%% Auto-generate efficient matlab function to lie derivatives
% FolderName = 'autogen_fun';
% if ~exist(FolderName,'dir')
%     warning('directory does not exist:%s\n It will be created.', FolderName)
%     mkdir(FolderName);
% end

