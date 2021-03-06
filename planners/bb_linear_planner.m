function [x_d] = bb_linear_planner(t,x)


    if(t< 10)
        dtheta_d = 0;
        theta_d = 0;
        
        dphi_d = 0;
        phi_d = 0;
        
    elseif(t<20)
       
        dphi_d = 0;
        phi_d = 0*pi/180;
        
        xpos = 1; % linear x position[m] xpos = r*(theta+phi)
        dtheta_d = 0;
        theta_d = xpos/0.105838037 - phi_d;%dtheta_d*(t-10);
        %theta_d = 2/0.105838037*0;
    else
        dtheta_d = 0;
        theta_d = 0;
        
        dphi_d = 0;
        phi_d = 0;
    end
    
    x_d = [theta_d; phi_d; dtheta_d; dphi_d];
end

