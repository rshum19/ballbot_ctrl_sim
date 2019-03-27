function [u,uparts] = PID_PD_controller(t,x,x_d,params)
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
    theta_d = x_d.ref_traj(1);
    phi_d = x_d.ref_traj(2);
    dtheta_d = x_d.ref_traj(3);
    dphi_d = x_d.ref_traj(4);
    
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
    K_ilc = [0.004 0.009 0 0];
    
    % Augmented desired lean angle
	phi_des = phi_d + K_ilc*err;
    phi_des = 0;
    
    %% ----------------------------------------------------------
    %   Inner loop control
    % ----------------------------------------------------------- 
    % Aim: I/O feedback linearized controller with PD to track desired lean angles
    % to balance and navigate
    
    % Output function    
    Kp = -10;
	Kd = 0*10;
	Ki = 80;
    %Klqr =  [-10.0000  972.7276  -16.2093  275.4142];
    %Klqr = [-1.0000  228.3256   -1.9662   55.4484];
    Klqr = 1.0e+03 * [-0.0316    1.0079   -0.0231    0.2855];
    Klqr = 1.0e+03 *[-0.0100    0   -0.0376   0];
    
	phie = phi - phi_des;
	%u = Kp*phie - Kd*(dtheta-dtheta_d);% + Ki*phie_agg;
    err2 = [theta-0; dtheta - 0;...
		    phi-0; dphi-0];
    u = Klqr*err2;
	%phie_agg = phie_agg + phie;
	%phie_hist(i) = phie;
    
    % Bound input
	if(abs(u)>params.u_max)
 		u = sign(u)*params.u_max;
	end
    %% ----------------------------------------------------------
    %   Pack controller relevant info
    % ----------------------------------------------------------- 
    uparts.u_ff = 0;
    uparts.u_fb = 0;
    uparts.mu = 0;
    uparts.phi_des = phi_des;
    uparts.kerr = 0;
    uparts.phi_d = phi_d;
    
    uparts.V_eps = 0;
    uparts.d1 = 0;
    uparts.psi0 =0;
    uparts.psi1 = 0;
end
