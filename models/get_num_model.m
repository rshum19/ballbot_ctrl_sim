clear all; close all; clc;

model = 'syms_model_ballbot2D.mat';
load(model);

[params, unpacked] = get_ballbot2D_model_params();
%[params] = get_shmoo_model_params(2);
load(unpacked);

Anum = double(subs(A_lin));
Bnum = double(subs(B_lin));
