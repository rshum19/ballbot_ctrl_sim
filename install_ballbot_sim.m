%% Adding path of folders to run simulation 
fprintf('Installing ballbot_ctrl_sim package...');
addpath(genpath('controllers'));  
addpath(genpath('examples'));  
addpath(genpath('models'));  
addpath(genpath('planners'));  
addpath(genpath('plot'));  
addpath(genpath('tools'));  
fprintf('DONE \n');

% Look for ballbot_viz package
fprintf('Installing ballbot_viz package...');
addpath(genpath('../ballbot_viz/'));
fprintf('DONE \n');