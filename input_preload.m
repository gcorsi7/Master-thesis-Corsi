% -------------
% input_preload
%
% This input file gives the commanded rotational velocity of the simulator
% to obtain a rotation of the angle theta_tot in time t_tot with 
% the parameter n, which is the percentage of acceleration time t_acc
% with respect to the total time t_tot

n = 0.5;
theta_tot = 270*pi/180; % radians;
t_tot = 20;             % seconds;

vel_slope = theta_tot/(n*t_tot^2*(1-n)); % obtained integrating the equations of motion

t_acc = n*t_tot;
t_cons = (1-2*n)*t_tot;

t0 = 0;
t1 = t0 + t_acc;
t2 = t0 + t_acc + t_cons;
t3 = t0 + 2*t_acc + t_cons;