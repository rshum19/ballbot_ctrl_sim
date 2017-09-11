function [ x, f, g ] = EoM2CntrlAffine( D, H, B, q, dq )
%EoM2CntrlAffine Converts Eular Lagrange EoM into control affine
%representation.
% AUTHOR:   Roberto Shu
% LAST EDIT: 8/26/2017
%
% Converts equation of motion represented in the form:
%   D(q)*dqq + H(q,dq) = B*u + J(q)'*F_external     (1)
% into the form:
%   dx = f(x) + g(x)*u

% Inputs:
%   D:  Inertia-mass matrix
%   H:  Centrifugal, coriolis, and gravity vector   
%   B:  Input matrix
%   q,dq:  state varaibles and derivative vectors
%
% Outputs:
%   x: augemented state vector
%   f: dynamics matrix
%   g: inputs matrix
%

% Augmented state vector
x = [q;dq];

% Construct dynamics matrix
f = [dq; -D\H];
f = simplify(f);

% Construct input matrix
g = [zeros(size(dq)); D\B];
g = simplify(g);
end

