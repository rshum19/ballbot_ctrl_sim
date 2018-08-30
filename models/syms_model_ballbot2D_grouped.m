% FILENAME: syms_model_ballbot2D_grouped.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 2/2/2018
%
% DESCRIPTION:
% This script contains the equationsof motion of a 2D Ballbot with arm
% model. The equation of motion were derived using the script 
% syms_model_ballbot2D.m The equation simplification and grouping was done by hand.
% It uses a Coordinate scheeme based on [3] and uses a similar
% grouping of constant variables to simply notation. 
%   D(q)ddq + C(q,dq)dq + G(q) = B(q)u + J(q)'*F_external
%
%       D(q) = Mass matrix
%       C(q,dq) = Coriolis forces
%       G(q) = gravitational forces
%       B(q) = output matrix
%       J(q) = contact jacobian
%
% References:
% [3] Nagarajan, Umashankar, George Kantor, and Ralph L. Hollis. 
%     "Trajectory planning and control of an underactuated dynamically stable single 
%     spherical wheeled mobile robot." Robotics and Automation, 2009. ICRA'09. IEEE 
%     International Conference on. IEEE, 2009.

% Initializing symbolic variables
syms th phi alpha dth dphi dalpha real
syms grav real
syms I_ball I_body I_arm M_ball M_body M_arm r L l_arm L_armjoint real;
syms tau tau_a real;

%% Name system model
modelName = 'planarBB_wArm_grouped';
% variables
q = [th; phi; alpha];
dq = [dth; dphi; dalpha];
u = [tau; tau_a];

params_vec = [M_body M_ball M_arm I_body I_ball I_arm L r L_armjoint l_arm grav];

% Constant variables
omega  = I_ball + (M_ball + M_arm + M_body)*r^2;
beta = M_body*r*L;
gamma = M_arm*r*L_armjoint;
psi = M_arm*r*l_arm;
nu = I_arm + M_arm*l_arm^2;
zeta = I_body + M_body*L^2 + M_arm*L_armjoint^2;

% Defining EoM matrix elements
% D(q)ddq + C(q,dq)dq + G(q) = B(q)u + J(q)'*F_external
D = [   omega,    omega+(beta+gamma)*cos(phi)+psi*cos(alpha-phi), psi*cos(alpha-phi);...
        omega+(beta+gamma)*cos(phi)+psi*cos(alpha-phi), omega+2*(beta+gamma)*cos(phi)-2*psi*cos(alpha-phi)-(2*gamma*psi*cos(alpha))/(M_arm*r^2)+zeta+nu, (gamma*psi*cos(alpha))/(M_arm*r^2)-nu+psi*(alpha-phi);...
        psi,    (gamma*psi*cos(alpha))/(M_arm*r^2)-nu+psi*cos(alpha-phi),    nu];
    
C = [   0,  psi*sin(alpha-phi)*(dalpha-dphi)-(beta+gamma)*sin(phi)*dphi,    -psi*sin(alpha-phi)*(dalpha-dphi);...
        0,  psi*sin(alpha-phi)*(dalpha-dphi)-(beta+gamma)*sin(phi)*dphi+(gamma*psi*sin(alpha)*dalpha)/(M_arm*r^2),   -(psi*sin(alpha-phi)+(gamma*psi*sin(alpha))/(M_arm*r^2))*(dalpha-dphi);...
        0,  (-gamma*psi*sin(alpha))/(M_arm*r^2)*dphi,  0];
    
G = [   0;...
        -(beta+gamma)*grav*sin(phi)/r - psi*grav*sin(alpha-phi)/r;...
        psi*grav*sin(alpha-phi)/r];
G = simplifyFraction(G);
    
B = [   1,  0;...
        0,  0;...
        0,  1];

% ODE Form (affine state space model)
% dx = f(x) + g(x)*u
x = [q;dq];
f = [dq; -D\(C*dq-G)]; f = simplify(f);
g = [zeros(size(B)); D\B]; g = simplify(g);

% Linearized system
A_lin = jacobian(f,x);
B_lin = jacobian(g*u,u);

A_lin = subs(A_lin,x,zeros(size(x)));
B_lin = subs(B_lin,u,zeros(size(u)));

% Output
x_ball = -r*th;
y_ball = r;
x_E = x_ball + L_armjoint*sin(phi) - L_arm*sin(-alpha+phi);
y_E = y_ball + L_armjoint*cos(phi) - L_arm*cos(-alpha+phi);


%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Auto-generate efficient matlab function to compute dynamics

%matlabFunction(f, 'File',fullfile('autogen_fun',strcat('autofun_f_',modelName)),'Vars',horzcat(q',dq',params_vec));
%matlabFunction(g, 'File',fullfile('autogen_fun',strcat('autofun_g_',modelName)),'Vars',horzcat(q',dq',params_vec));

matlabFunction(A_lin, 'File',fullfile('autogen_fun',strcat('autofun_A_lin_',modelName)),'Vars',params_vec);
matlabFunction(B_lin, 'File',fullfile('autogen_fun',strcat('autofun_B_lin_',modelName)),'Vars',params_vec);
