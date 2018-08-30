syms alpha beta gamma real
syms g r real;
syms th phi real;
syms tau real;

D = [alpha, alpha+beta; alpha+beta, alpha+gamma+2*beta];
G = [0; beta*g*phi/r];
u = [tau;0];