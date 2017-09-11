function [ dx ] = planarBB_wArm_dyn_wrap(t,x,u,params)

    
    theta = x(1);
    phi = x(2);
    alpha = x(3);
    dtheta = x(4);
    dphi = x(5);
    dalpha = x(6);
    
    tau = u(1);
    tau_a = u(2);
    
    dx = autofun_ddq_ode_planarBB_wArm(theta,phi,alpha,dtheta,dphi,dalpha,...
                        tau,tau_a,...
                        params.M_body,params.M_ball,params.M_arm,...
                        params.I_body,params.I_ball,params.I_arm,...
                        params.L,params.r,params.L_armjoint,params.l_arm,...
                        params.grav);
    
    
    
end

