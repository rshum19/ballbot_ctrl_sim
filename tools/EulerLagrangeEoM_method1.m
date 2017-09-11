function [D_mtx,C_mtx,G_vec,B_mtx] = EulerLagrangeEoM_method1(KE,PE,q,dq,Phi)
%EulerLagrangeEoM_method1 computes system dynamis using Euler Lagrange Method
% AUTHOR:   Roberto Shu
% LAST EDIT: 8/26/2017
% Inputs:
%   KE:     System's Kinetic Energy
%   PE:     System's Potential Energy
%   q:      symbolic state variables
%   dq:     symbolic state velocities/derivatives
%   Phi:    row vector of controlled state variables
%
% Outputs:
%   D_mtx*ddq + C_mtx*dq + G_vec = B_mtx*u
%

% Gravity vector
G_vec = simplify(jacobian(PE,q).');

% Mass-Inertia matrix
D_mtx = simplify(jacobian(KE,dq).');
D_mtx = simplify(jacobian(D_mtx,dq));

% Coriolis and centrifugal matrix
syms C_mtx g real
n=max(size(q));
for k=1:n
    for j=1:n
        C_mtx(k,j) = 0*g;
        for i=1:n
            C_mtx(k,j) = C_mtx(k,j)+ 1/2*(diff(D_mtx(k,j),q(i)) + ...
                diff(D_mtx(k,i),q(j)) - ...
                diff(D_mtx(i,j),q(k)))*dq(i);
        end
    end
end
C_mtx = simplify(C_mtx);

% Input matrix  
B_mtx = simplify(jacobian(Phi,q));
B_mtx = B_mtx';

end