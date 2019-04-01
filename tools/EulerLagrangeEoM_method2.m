function [ D, H, B ] = EulerLagrangeEoM_method2( Lag, q, dq, ddq, gamma)
%EulerLagrangeEoM_method2 computes system dynamis using Euler Lagrange Method
% AUTHOR:   Roberto Shu
% LAST EDIT: 8/26/2017
%
% Inputs:
%   Lag:    Langrange equation of system
%   q:      symbolic state variables
%   dq:     symbolic state velocities/derivatives
%   gamma:    row vector of controlled state variables
%
% Outputs:
%   D(q)*dqq + H(q,dq) = B*u + J(q)'*F_external
%

LHS = jacobian(jacobian(Lag, dq)', [q;dq])*[dq;ddq]  -  jacobian(Lag, q)' ;

% Mass-Inertia matrix
D = jacobian(LHS, ddq);
D = simplify(D);

% Coriolis, centrifugal and gravity  matrix
H = simplify(LHS - D*ddq) ;

% Input matrix  
B = simplify(jacobian(gamma,q));
B = B';

% External force Jacobian
J = [];

end

