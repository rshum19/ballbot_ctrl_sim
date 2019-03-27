function sim_ballbot2D(X0, sim_params, sim_opts)
    close all;
    
    %% ----------------------------------------------------------
    %   Input verification
    % -----------------------------------------------------------
    % Check for inputs, if not inputs set default values
    if(nargin < 3)
        
        % Simulation options
        sim_opts.ode_type = 0;
        sim_opts.animate = 1;
        sim_opts.animate_fun = @plot_bb;
        sim_opts.save = 0;
        sim_opts.plot = 0;
        
        % Simulation parameteres
        if(nargin < 2)
            sim_params.model_params = get_model_params;
            sim_params.t_range = [0 10];
            sim_params.ctrl_params.c3 = 0.5;
            sim_params.ctrl_params.epsilon = 0.8;
            sim_params.ctrl = @RES_CLFQP_controller;
        
            % Intial condition
            if(nargin < 1)
                X0 = [0; -2*pi/180; 0; 0] ;
            end
        
        end
    end
    
    %% Load constants and perofm one time setup
    model_params = sim_params.model_params;
   
    % one-time setup
    %ctrl = initial_setup(x0, model_params);
    
    %% ----------------------------------------------------------
    %   Simulation
    % -----------------------------------------------------------
    ode_opts = odeset('Events',@bbfallEvent2) ;
    if(sim_opts.ode_type == 0)
        [t_out X_out] = ode45(@(t,x)odefun_dynamicsBB(t,x,sim_params), sim_params.t_range, X0,ode_opts) ;
    elseif(sim_opts.ode_type == 1)
        [t_out X_out] = ode23(@(t,x)odefun_dynamicsBB(t,x,sim_params), sim_params.t_range, X0,ode_opts);
    else
        [t_out X_out] = ode23s(@(t,x)odefun_dynamicsBB(t,x,sim_params), sim_params.t_range, X0,ode_opts) ;
    end
    
    % Record control inputs
    u_out = zeros(length(t_out),1);
    X_d_out = zeros(length(t_out),size(X0,1));
    u_fb = zeros(length(t_out),2);
    u_ff = zeros(length(t_out),1);
    psi0 = zeros(length(t_out),1);
    psi1 = zeros(length(t_out),1);
    phi_d = zeros(length(t_out),1);
    kerr = zeros(length(t_out),1);
    V_eps = zeros(length(t_out),1);
    d1 = zeros(length(t_out),1);
    phi_des = zeros(1,1);
    for j = 1:length(t_out)
        [~, u, xd] = odefun_dynamicsBB(t_out(j),X_out(j,:)',sim_params);
        u_out(j,:) = u';
        u_ff(j,:) = xd.uparts.u_ff;
        u_fb(j,:) = xd.uparts.u_fb;
        psi0(j,:) = xd.uparts.psi0;
        psi1(j,:) = xd.uparts.psi1;
        phi_d(j,:) = xd.uparts.phi_d;
        kerr(j,:) = xd.uparts.kerr;
        V_eps(j,:) = xd.uparts.V_eps;
        d1(j,:) = xd.uparts.d1;
        x_d = xd.ref_traj;
        phi_des(j,:) = xd.phi_des;
        X_d_out(j,:) = x_d';
    end
    
    %% ----------------------------------------------------------
    %   SAVE MODEL
    % ----------------------------------------------------------- 
    save_opts = sim_opts.save_opts;
    if(save_opts.save)
        
        data = packResults( X0, t_out, X_out, u_out, model_params, sim_params);
        notes = save_opts.notes;
        
        if ~exist('sim_opts.overWrite','var')
            save_opts.overWrite = 0;
        end
        
        save_results( save_opts.folderName, save_opts.fileName, save_opts.overWrite,data,notes)
    end
    
    %% ----------------------------------------------------------
    %   Animation
    % -----------------------------------------------------------
    if(sim_opts.animate)
        Anim.speed = sim_opts.animate_speed;
        Anim.plotFunc = sim_opts.animate_fun;
        animate(t_out,X_out,Anim);
    end
    
    %% ----------------------------------------------------------
    %   Plots
    % -----------------------------------------------------------
    if(sim_opts.plot)
        if(isempty(sim_opts.plot_fun))
            figure(2)
            % Ball angle
            subplot(3,2,1)
            plot(t_out,X_out(:,1)*180/pi)
            xlabel('Time (sec)')
            ylabel('\theta (deg)')

            % Body angle
            subplot(3,2,2)
            plot(t_out,X_out(:,2)*180/pi,t_out,phi_des*180/pi)
            xlabel('Time (sec)')
            ylabel('\psi (deg)')

            % Ball angular velocity
            subplot(3,2,3)
            plot(t_out,X_out(:,3)*180/pi)
            xlabel('Time (sec)')
            ylabel('$\dot{\theta}$ (deg)','interpreter','latex')

            % Body angular velocity
            subplot(3,2,4)
            plot(t_out,X_out(:,4)*180/pi)
            xlabel('Time (sec)')
            ylabel('$\dot{\psi}$ (deg)','interpreter','latex')

            % Ball center x-translation
            subplot(3,2,5)
            plot(t_out,model_params.r*(X_out(:,1)+X_out(:,2)))
            xlabel('Time (sec)')
            ylabel('$x$ (m)','interpreter','latex')

            % Ball center linear velocity
            subplot(3,2,6)
            plot(t_out,model_params.r*(X_out(:,3)+X_out(:,4)))
            xlabel('Time (sec)')
            ylabel('$\dot{x}$ (m/s)','interpreter','latex')

            % Control input
            figure(3)
            subplot(3,1,1)
            plot(t_out,u_out);
            xlabel('Time (sec)');
            ylabel('Input (Nm)');
            
            subplot(3,1,2)
            plot(t_out,u_fb);
            ylabel('u_fb')
            
            subplot(3,1,3)
            plot(t_out,u_ff);
            ylabel('u_ff');
            
            
            figure(4)
            subplot(3,1,1)
            plot(t_out,phi_des*180/pi);
            ylabel('phi_des')
            
            subplot(3,1,2)
            plot(t_out,phi_d*180/pi);
            ylabel('phi_d')
            
            subplot(3,1,3)
            plot(t_out,kerr*180/pi);
            ylabel('kerr');
            
            figure(5)
            subplot(2,1,1)
            plot(t_out,psi0);
            ylabel('psi0');
            subplot(2,1,2);
            plot(t_out,psi1);
            ylabel('psi1');
            
            figure
            plot(t_out,V_eps);
            ylabel('V_eps');
            
            figure
            plot(t_out,d1);
            ylabel('d1');
        else
            sim_opts.plot_fun(t_out,X_out,X_d_out,u_out,u_fb,phi_des,sim_params.model_params);
        end
    end
    
end

% Function to describe the dynamics of the rocket
function [dx u x_d] = odefun_dynamicsBB(t, x, sim_params)
    
    % Print integration time step
    %t
    
    % Call reference trajectory generators
    x_d.ref_traj = sim_params.ctrl_ref_traj(t,x);
    
    % Call controller
    [u,uparts] = sim_params.ctrl(t,x,x_d,sim_params.ctrl_params);
    x_d.phi_des = uparts.phi_des;
    x_d.uparts = uparts;
    
    % Call dynamic equation (dx)
    dx = sim_params.model(t,x,u);
    
    % check for fall
    if (abs(x(2)) >= pi/2)
      dx = zeros(size(x));
    end
end

function [value,isterminal,direction] = bbfallEvent2( t, x, sim_params )

value = abs(x(2)) - pi/2;
isterminal  = 1;
direction = 1;

end

function data = packResults( X0, t_out, X_out, u_out, model_params, sim_params )
    
    data.model_params = model_params;
    data.X0 = X0;
    data.t_out = t_out;
    data.X_out = X_out;
    data.u_out = u_out;
    data.sim_params = sim_params;
end
