function plots_planarBB_wArm(t_out,X_out,X_d_out,u_out,phi_des,kerr,V_eps,psi0,psi1,model_params)

        figure(2)
        % Ball angle
        subplot(4,2,1)
        plot(t_out,X_out(:,1)*180/pi,t_out,X_d_out(:,1)*180/pi)
        xlabel('Time (sec)')
        ylabel('\theta (deg)')

        % Body angle
        subplot(4,2,3)
        plot(t_out,X_out(:,2)*180/pi,t_out,X_d_out(:,2)*180/pi,t_out,phi_des*180/pi)
        xlabel('Time (sec)')
        ylabel('\psi (deg)')
        
        % Arm angle
        subplot(4,2,5)
        plot(t_out,X_out(:,3)*180/pi,t_out,X_d_out(:,3)*180/pi)
        xlabel('Time (sec)')
        ylabel('\alpha (deg)')
        
        % Ball angular velocity
        subplot(4,2,2)
        plot(t_out,X_out(:,4)*180/pi,t_out,X_d_out(:,4)*180/pi)
        xlabel('Time (sec)')
        ylabel('$\dot{\theta}$ (deg)','interpreter','latex')
    
        % Body angular velocity
        subplot(4,2,4)
        plot(t_out,X_out(:,5)*180/pi,t_out,X_d_out(:,5)*180/pi)
        xlabel('Time (sec)')
        ylabel('$\dot{\psi}$ (deg)','interpreter','latex')

        % Arm angular velocity
        subplot(4,2,6)
        plot(t_out,X_out(:,6)*180/pi,t_out,X_d_out(:,6)*180/pi)
        xlabel('Time (sec)')
        ylabel('$\dot{\alpha}$ (deg)','interpreter','latex')
        
        % Ball center x-translation
        subplot(4,2,7)
        plot(t_out,model_params.r*(X_out(:,1)+X_out(:,2)))
        xlabel('Time (sec)')
        ylabel('$x$ (m)','interpreter','latex')
    
        % Ball center linear velocity
        subplot(4,2,8)
        plot(t_out,model_params.r*(X_out(:,4)+X_out(:,5)))
        xlabel('Time (sec)')
        ylabel('$\dot{x}$ (m/s)','interpreter','latex')

        % Control input
        figure(3)
        plot(t_out,u_out);
        xlabel('Time (sec)');
        ylabel('Input (Nm)');
        
        figure
        plot(t_out,kerr)
        ylabel('kerr')
        
        figure
        subplot(2,1,1)
        plot(t_out,psi0);
        ylabel('psi0');
        subplot(2,1,2);
        plot(t_out,psi1);
        ylabel('psi1');

        figure
        plot(t_out,V_eps);
        ylabel('V_eps');

end

