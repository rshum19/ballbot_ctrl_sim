%% Script to derive a PD-LQR controller 
clear all; close all; clc;


%% Load system dynamic model
controllerName = 'PD_LQR';
modelName = 'ballbot2D';
addpath('../../models')
load(strcat('syms_model_',modelName,'.mat'));

% Model Parameters values
[params, unpacked_params] = get_ballbot2D_model_params();
load(unpacked_params);

%% Ballbot 2D Model
% Define symbolic variables
syms phi theta dphi dtheta ddphi ddtheta tau real % State variables

% State vector
q = [theta; phi];
dq = [dtheta; dphi];
X = [q;dq];
u = tau;


%% Linearize System
Alin = jacobian(dX,X);
Blin = jacobian(dX,u);

%% Subsitute params values into linear dynamics
phi = 0; dphi = 0;
Alin_num = double(subs(Alin));
Blin_num = double(subs(Blin));


%% Calculate LQR Gains
Q = [1, 0, 0, 0;
       0, 100, 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 100]; 

R = eye(size(u));
     
Klqr = lqr(Alin_num,Blin_num,Q,R);


