% --------
% RWSizing
% 
% Script used for a first approximation sizing of the reaction wheel
%
% The equation implemented are based mainly on:
% "Analysis of the motion of a satellite-reaction wheel assembly optimized
% for weight and power", C.R. Hayleck, NASA, December 1965
% 

n = [0:0.01:0.5];       
theta_tot = 270*pi/180; % radians
t_tot = 20;             % seconds
I_v = 0.116;            % kg*m^2

for i = 1:length(n)
    
    T(i) = (theta_tot*I_v)/((n(i)-n(i)^2)*t_tot^2); % torque (constant)
    h_max(i) = T(i)*n(i)*t_tot; % angular momentum (maximum value) 
end

figure(1)
plot(n,T,'linewidth',1.8)
grid on;
title('Torque vs n')
xlabel('n');
ylabel('T');

figure(2)
plot(n,h_max,'linewidth',1.8)
grid on;
title('Max angular momentum vs n')
xlabel('n');
ylabel('h_{max}');

figure(3)
plot(T,h_max,'linewidth',1.8)
grid on;
title('Torque vs max angular momentum')
xlabel('T');
ylabel('h_{max}');


%% ---Weight/Inertia---
% Assumption: the wheel is a solid disc of radius r

g = 9.81;
r = [0.1:0.01:0.6]; % different values for the radius
% omega is the maximum velocity of the wheel in the simulink simulation. 
% It change every time according to slew maneuver 
omega = 1800; % rpm
omega = omega*2*pi/60; % rad/s

for i = 1:length(n)
    for j = 1:length(r)
        
        W(j,i) = (g*2*h_max(i))/(omega*r(j)^2); % Weight
        J(i) = h_max(i)/(3000*2*pi/60); % Inertia: the denominator is 3000 rpm (max value assumed for the wheel) converted to rad/s
    end
end

figure(4)
surf(n, r, W)
grid on;
title('Weight vs n and r')
xlabel('n');
ylabel('r');
zlabel('W');

figure(5)
plot(n, J, 'linewidth', 1.8)
grid on;
title('Inertia vs n')
xlabel('n');
ylabel('Inertia J');


%% ---Power---
eta = 1; % efficiency, optional parameter
t1 = linspace(0,10,51);
t2 = linspace(10,20,51);
t = [t1 t2];
% evaluate max value of inertia
Jfix = J(end);
for i = 1:length(n)
    
        P_max_omegafix(i) = T(i)*omega; % omega fixed 
        
        omega_var(i) = h_max(i)/Jfix; % inertia fixed, omega variable
        P_max_inertiafix(i) = T(i)*omega_var(i);
        
        % P evaluated only to check the profile of the power 
        % for the case n=0.5
        P(i) = T(end)*omega*t1(i)/10/eta;
        P(length(t1)+i) = T(end)*omega*((20 - t2(i))/10)/eta;
end

figure(6)
plot(n,P_max_omegafix,'linewidth',1.8)
grid on;
title('Power vs n for fixed values of \omega_w')
xlabel('n');
ylabel('P_{max} [W]');

figure(7)
plot(n,P_max_inertiafix,'linewidth',1.8)
grid on;
title('Power vs n for fixed values of inertia')
xlabel('n');
ylabel('P_{max} [W]');

figure(8)
plot(t,P,'linewidth',1.8)
grid on;
title('Power vs t for n=0.5 (plotted only to check power profile)')
xlabel('t');
ylabel('P [W]');




