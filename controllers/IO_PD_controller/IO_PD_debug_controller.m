function [u,uparts] = IO_PD_debug_controller(t,x,x_d,params)
% IO_PD_controller  I/O feedback linearized controller with an outer loop PD
% AUTHOR:   Roberto Shu
% LAST EDIT: 8/26/2017    
    %% ----------------------------------------------------------
    %   Unpack actual and desired state vectors
    % -----------------------------------------------------------    
    % State variables
    theta = x(1);
    phi = x(2);
    dtheta = x(3);
    dphi = x(4);
    
    % Desired state
    theta_d = 0;
    phi_d = 0;
    dtheta_d = 0;
    dphi_d = 0;
    
    %% ----------------------------------------------------------
    %   Outer loop control
    % ----------------------------------------------------------- 
    % Aim: get a desired lean angle (phi_des) to track desired ball
    % position using a simple PD controller
    
    % Error state vector
	err = [theta-theta_d; dtheta - dtheta_d;...
		phi-phi_d; dphi-dphi_d];
	
    % Control gains 
    %K_ilc = [0.01 0.015 0 0];
    %K_ilc = [0.004 0.009 0 0];
    K_ilc = [0.03 0.002 0 0];
    
    % Augmented desired lean angle
    phi_sk = K_ilc*err;
	phi_des = phi_d - phi_sk; 
    phi_des = 0;
    
    %% ----------------------------------------------------------
    %   Inner loop control
    % ----------------------------------------------------------- 
    % Aim: I/O feedback linearized controller with PD to track desired lean angles
    % to balance and navigate
    
%     % Output function
%     y = phi - phi_des;
%     dy = dphi - dphi_d;
%     y_vec = [y;dy];
%     
%     % Output function lie derative 
%     [LgLfh, Lf2h] = autofun_LieDerv_ballbot2D_IO_PD(theta,phi,dtheta,dphi,phi_des,dphi_d);
% 
%     % Control gains 
%     K = [7.0711    4.9135];
%     %K = [22.3607   12.0300];
%     %K = [50     15];
%     
%     % Feedfoward term
%     u_ff = -(LgLfh)^(-1)*Lf2h;
%     
%     % PD term
%     mu = -K*y_vec;
%     u_fb = (LgLfh)^(-1)*mu;
%     
%     % Full I/O controller
%     uPID = u_ff + u_fb;
    
    % Output function    
    Kp = 2000;
	Kd = 5;
	Ki = 80;
    
	phie = phi - phi_des;
	u = Kp*phie + Kd*(dtheta-dtheta_d);% + Ki*phie_agg;
    
    % Integrate error
    %phie_agg = phie_agg + phie;
    
    % Bound input
	if(abs(u)>params.u_max)
 		u = sign(u)*params.u_max;
	end
    %% ----------------------------------------------------------
    %   Pack controller relevant info
    % ----------------------------------------------------------- 
    uparts.u_ff = u;
    uparts.u_fb = u;
    uparts.mu = 0;
    uparts.phi_des = phi_des;
    uparts.kerr = K_ilc*err;
    uparts.phi_d = phi_d;
    
    uparts.V_eps = 0;
    uparts.d1 = 0;
    uparts.psi0 =0;
    uparts.psi1 = 0;
end
