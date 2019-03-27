function plots_planarBB_simple(t_out,X_out,X_d_out,u_out,u_fb,phi_des,model_params)

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
    subplot(2,1,1)
    plot(t_out,u_out);
    xlabel('Time (sec)');
    ylabel('U_IO (Nm)');

    subplot(2,1,2)
    plot(t_out,u_fb);
    ylabel('u_PID (Nm)')

end
