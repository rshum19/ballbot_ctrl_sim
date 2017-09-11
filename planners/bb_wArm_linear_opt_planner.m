function [x_d] = bb_wArm_linear_opt_planner(time,x)

    load('opt_traj.mat')
    if(time< 10)
        dtheta_d = 0;
        theta_d = 0;
        
        dphi_d = 0;
        phi_d = 0;
        
        alpha_d = 0;
        dalpha_d = 0;

    elseif(time >=10 & time <=19)
        theta_d = interp1(t,z(1,:),(time-10));
        phi_d = interp1(t,z(2,:),(time-10));
        alpha_d = interp1(t,z(3,:),(time-10));
        dtheta_d = interp1(t,z(4,:),(time-10));
        dphi_d = interp1(t,z(5,:),(time-10));
        dalpha_d = interp1(t,z(6,:),(time-10));
    else
        dtheta_d = 0;
        theta_d = 0;
        
        dphi_d = 0;
        phi_d = -3*pi/180;
        
        alpha_d = z(3,end);
        dalpha_d = 0;
    end
    x_d = [theta_d; phi_d; alpha_d; dtheta_d; dphi_d; dalpha_d];

end

