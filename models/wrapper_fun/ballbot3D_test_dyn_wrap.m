function [ dx ] = ballbot3D_test_dyn_wrap(t,x,u,params)

    % Unpack state variables
    thetax = x(1);
    thetay = x(2);
    phix = x(3); %phi_x
    phiy = x(4); %phi_y
    dthetax = x(5);
    dthetay = x(6);
    dphix = x(7); %dphi_x
    dphiy = x(8); %dphi_y
          
    % Unpack control inputs
    tau_x = u(1);
    tau_y = u(2);
    
    Mw  = 2.437; %kg
    Mb  = 51.663126; %kg
    r = 0.105838037; %m
    g = 9.81; %m/s^2
    l = 0.69; %m
    Ixx = 12.5905; %kg m^2
    Iyy = 12.5905; %kg m^2
    Izz = 0.6644; %kg m^2
    Iw = 0.0174; %kg m^2
    
    [H,C,G] = autofun_ddq_ode_test(thetax,thetay,phix,phiy,dthetax,dthetay,dphix,dphiy,...
                                   Mw,Mb,...
                                   Ixx,Iyy,Izz,Iw,...
                                   l,r,g);
    % H*ddq + C*dq + G 
    dq = [dthetax; dthetay; dphix; dphiy];
    ddq = H\(-C*dq - G');
    
    dx = [dq;ddq];
    
end

