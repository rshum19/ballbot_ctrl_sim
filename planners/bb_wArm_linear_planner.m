function [x_d] = bb_wArm_linear_planner(t,x)


    if(t< 10)
        dtheta_d = 0;
        theta_d = 0;
        
        dphi_d = 0;
        phi_d = 0;
        
        alpha_d = 0;
        dalpha_d = 0;
        
    elseif(t>=10 & t<= 20)
        dtheta_d = 0;
        theta_d = 0;

        dphi_d = 0;
        phi_d = 0*pi/180;
        
        dalpha_d =9*pi/180;
        alpha_d = dalpha_d*(t-10);
    else
        
        dtheta_d = 0;
        theta_d = 0;

        dphi_d = 0;
        phi_d = 0*pi/180;
        
        dalpha_d = 0;
        alpha_d = 9*pi/180*10;

    end
    x_d = [theta_d; phi_d; alpha_d; dtheta_d; dphi_d; dalpha_d];
end

