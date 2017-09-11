function [ dx ] = planarBB_wCoMoffset_dyn_wrap(t,x,u,params)

    
    theta = x(1);
    phi = x(2);
    dtheta = x(3);
    dphi = x(4);
    
    tau = u(1);
    
    beta = params.beta;
    dx = autofun_ddq_ode_planarBB_wCoMoffset(theta,phi + beta,dtheta,dphi,...
                        tau,...
                        params.M_body,params.M_ball,...
                        params.I_body,params.I_ball,...
                        params.L,params.r,beta,...
                        params.grav);
    
    
    
end

