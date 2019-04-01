function [u,uparts] = PD_LQR_controller(t,x,x_d,params)
% LQR_controller  LQR controller 
% AUTHOR:   Roberto Shu
% LAST EDIT: 3/31/2019    
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
    
    % Error state vector
    err = [theta-theta_d; phi-phi_d; dtheta-dtheta_d;dphi-dphi_d];

    phi_des = 0;
    
    %% ----------------------------------------------------------
    %   Inner loop control
    % -----------------------------------------------------------     
    
    Klqr = [-1.0000 -173.1954   -2.0268  -48.6683]; % LQR Gains for uma's model
    
    % Compute input via Full State Feedback control law
    u = -Klqr*err;
    
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
