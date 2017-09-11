%% Prepare workspace
clear all; close all; clc;

%% Initial conditions
theta = 0;
phi = 2 * pi/180;
beta = 0;
dtheta = 0;
dphi = 0*pi/180;
dbeta = 0*pi/180;

X0 = [theta;phi;dtheta;dphi];

%% Simulation Options
sim_opts.ode_type = 1;
sim_opts.animate = 0;
sim_opts.animate_fun = @draw_bb;
sim_opts.animate_speed = 1;
sim_opts.save_opts.save = 0;
sim_opts.save_opts.folderName = 'results';
sim_opts.save_opts.fileName = 'sim_planarBB_wCLFQP_run_';
sim_opts.save_opts.overWrite = 0;
sim_opts.save_opts.notes = '';

sim_opts.plot = 1;
sim_opts.plot_fun = [];

%% Simulation Parameters
% Model definition
sim_params.model_params = get_ballbot2D_model_params;
sim_params.model = @(t,x,u)ballbot2D_dyn_wrap(t,x,u,sim_params.model_params);
%sim_params.model = @(t,x,u)planarBB_wCoMoffset_dyn_wrap(t,x,u,sim_params.model_params);

% Controller defintion
sim_params.ctrl_params.u_max = 10;
sim_params.ctrl = @IO_PD_controller;

% Planner definition
sim_params.ctrl_ref_traj = @(t,x)bb_linear_planner(t,x);

% Simulation time
dt = 1/100;

sim_t_i = 0;
sim_t_f = 20;
sim_params.t_range = sim_t_i:dt:sim_t_f;

%% Call simulation function
sim_ballbot2D(X0,sim_params,sim_opts)