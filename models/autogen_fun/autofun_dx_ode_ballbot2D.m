function dx = autofun_dx_ode_ballbot2D(theta,phi,dtheta,dphi,tau,M_body,M_ball,I_body,I_ball,L,r,grav)
%AUTOFUN_DX_ODE_BALLBOT2D
%    DX = AUTOFUN_DX_ODE_BALLBOT2D(THETA,PHI,DTHETA,DPHI,TAU,M_BODY,M_BALL,I_BODY,I_BALL,L,R,GRAV)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    26-Aug-2017 17:30:31

t2 = r.^2;
t3 = L.^2;
t4 = M_body.^2;
t5 = cos(phi);
t6 = I_ball.*I_body;
t7 = t2.*t3.*t4;
t8 = I_ball.*M_body.*t3;
t9 = I_body.*M_ball.*t2;
t10 = I_body.*M_body.*t2;
t11 = M_ball.*M_body.*t2.*t3;
t12 = t5.^2;
t18 = t2.*t3.*t4.*t12;
t13 = t6+t7+t8+t9+t10+t11-t18;
t14 = 1.0./t13;
t15 = dphi.^2;
t16 = M_ball.*t2;
t17 = M_body.*t2;
t19 = sin(phi);
t20 = I_ball.*grav;
t21 = M_ball.*grav.*t2;
t22 = M_body.*grav.*t2;
dx = [dtheta;dphi;t14.*tau.*(I_ball+I_body+t16+t17+M_body.*t3-L.*M_body.*r.*t5.*2.0)-L.*M_body.*t14.*t19.*(t20+t21+t22+I_body.*r.*t15+M_body.*r.*t3.*t15-L.*M_body.*grav.*r.*t5-L.*M_body.*t2.*t5.*t15);-t14.*tau.*(I_ball+t16+t17-L.*M_body.*r.*t5)+L.*M_body.*t14.*t19.*(t20+t21+t22-L.*M_body.*t2.*t5.*t15)];