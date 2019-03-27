function dx = statespace_planarBB_uma(t,x,u)

% State vector
q = [x(1); x(2)];
dq = [x(3); x(4)];
%u = x(5);

theta = q(1);
phi = q(2);

dtheta = dq(1);
dphi = dq(2);

% Ballbot Parameters
Mw  = 2.437; %kg
Mb  = 51.663126; %kg
r = 0.105838037; %m
g = 9.81; %m/s^2
l = 0.69; %m
Ib = 12.5905; %kg m^2
Iw = 0.0174; %kg m^2


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

% check for fall
if (abs(phi) >= pi/2)
      dq(1) = 0;
      ddq(1) = 0;
      dq(2) = 0;
      ddq(2) = 0;
end

dx = [dq;ddq];
end