%% Test IO-PD controller

x = [   0;  0;  0;  0];
x_d = [ 0;  0;  0;  0];

[u,uparts] = IO_PD_controller_c(x,x_d);