


r = 0.105838037; %m
L = 0.69; %m
larm = 0.457; %m

y = -0.69;
x = 0.457;

phi2 = acos((x^2+y^2-L^2 -larm^2)/(2*L*larm));

phi1 = atan(y/x) + atan((larm*sin(phi2))/(L + larm*cos(phi2)));

X = [0 phi1 - pi/2 phi2];
time = 0;
draw_bb_wArm(time,X)