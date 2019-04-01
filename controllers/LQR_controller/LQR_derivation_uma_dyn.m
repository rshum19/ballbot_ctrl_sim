%% Script to derive a PD-LQR controller 
clear all; close all; clc;


%% Load system dynamic model
controllerName = 'PD_LQR';
modelName = 'ballbot2Duma';
%addpath('../../models')
%load(strcat('syms_model_',modelName,'.mat'));

% Model Parameters values
%[params, unpacked_params] = get_ballbot2D_model_params();
%load(unpacked_params);

%% Ballbot 2D Model
% Ballbot Parameters
Mw  = 2.437; %kg
Mb  = 51.663126; %kg
r = 0.105838037; %m
g = 9.81; %m/s^2
l = 0.69; %m
Ib = 12.5905; %kg m^2
Iw = 0.0174; %kg m^2

% Define symbolic variables
syms phi theta dphi dtheta ddphi ddtheta tau real % State variables

% State vector
q = [theta; phi];
dq = [dtheta; dphi];
X = [q;dq];
u = tau;

alpha = Iw + (Mw + Mb)*r^2;
beta = Mb*l*r;
gamma = Ib + Mb*l^2;

M = [	alpha,	alpha + beta*cos(phi);...
		alpha + beta*cos(phi),	alpha + gamma + 2*beta*cos(phi)];

C = [ 	-beta*sin(phi)*dphi^2;...
		-beta*sin(phi)*dphi^2];

G = [	0;...
		-beta*g*sin(phi)/r];

U = [u; 0];

ddq = M \ (U - G - C);

% State Space representation
dX = [dq;ddq];

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
     
Klqr = lqr(Alin_num,Blin_num,Q,R)

%% Auto-generate efficient matlab function to lie derivatives
% FolderName = 'autogen_fun';
% if ~exist(FolderName,'dir')
%     warning('directory does not exist:%s\n It will be created.', FolderName)
%     mkdir(FolderName);
% end

