% Need to remove the yaw
% [1] Spong, Mark W., and M. Vidyasagar. Robot Dynamics and Control. Wiley, 2004.
%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------
syms alpha beta gamma dalpha dbeta dgamma  ddgamma ddbeta ddalpha real
syms theta_x theta_y dtheta_x dtheta_y ddtheta_x ddtheta_y real
syms M_body M_ball I_body I_ball grav L r real
syms tau_x tau_y real

% Name system model
modelName = 'ballbot3D';

% Configuration coordintes and velocities
% theta and phi are in radias, positive counter-clockwise
q = [theta_x; theta_y; gamma; beta; alpha];
dq = [dtheta_x; dtheta_y; dgamma; dbeta; dalpha];
ddq = [ddtheta_x; ddtheta_y; ddgamma; ddbeta; ddalpha];
u = [tau_x; tau_y];

params_vec = [M_body M_ball I_body I_ball L r grav];
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
V_body = [dgamma; dbeta; dalpha];

% Net rotation
R = Rz * Ry * Rx;

% Linear position and velocity of ball w.r.t. Frame {0}
P_ball = r * [  theta_x;    theta_y;    0];
V_ball = r * [  dtheta_x;    dtheta_y;   0];

% Body angular velocity w.r.t Frame {1}
w_15 = [   0;  0;  dalpha] + Rz*[  0;  dbeta;  0] + Rz*Ry*[    dgamma; 0;  0];

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
KE_ball = 0.5*M_ball*V_ball'*V_ball + 0.5*I_ball*(dtheta_x^2 + dtheta_y^2);
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
B = [1 0; 0 1; 0 0; 0 0; 0 0];

% ODE Form (affine state space model)
% dx = f(x) + g(x)*u
x = [q;dq];
f = [dq; -D\H];
g = [zeros(size(B)); D\B];