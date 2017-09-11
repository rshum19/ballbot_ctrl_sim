M_ballfunction [params,unpacked_params] = get_ballbot2D_model_params()

    params.M_ball  = 2.437; %kg
    params.M_body  = 51.663126; %kg
    params.M_arm = 3; %kg
    params.r = 0.105838037; %m
    params.grav = 9.81; %m/s^2
    params.L = 0.69; %m
    params.L_armjoint = 0.9; %m
    params.l_arm = 0.457; %m
    params.I_body = 12.5905; %kg m^2
    params.I_ball = 0.0174; %kg m^2
    params.I_arm = params.M_arm*params.l_arm^2;
    params.beta = 0*pi/180;
    
    
    % Unpacked parametrs 
    M_ball = params.M_ball; %kg
    M_body = params.M_body; %kg
    M_arm = params.M_arm; %kg
    r = params.r; %m
    grav = params.grav; %m/s^2
    L = params.L; %m
    L_armjoint = params.L_armjoint; %m
    l_arm = params.l_arm; %m
    I_body = params.I_body; %kg m^2
    I_ball = params.I_ball; %kg m^2
    I_arm = params.I_arm;
    
    unpacked_params = 'model_params.mat';
    
    save(unpacked_params,...
         'M_ball', 'M_body', 'M_arm', 'r', 'grav',...
          'L', 'L_armjoint', 'l_arm',...
          'I_body', 'I_ball', 'I_arm');
end

