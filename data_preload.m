% ------------
% data_preload
%
% Numerical data for air bearing simulation with 3 reaction wheels

% Reaction Wheel and Motor data
Ke = 0.008;    % V/rad/s
Kt = 0.00797;  % N*m/A
L = 25e-6;     % H
R = 0.73;      % Ohm
b = 1.05e-6;   % N*m/rad/s
J = 1.74e-4 + 5.1e-7;    % kg*m^2


I_w = J*eye(3);
I_w_inv = inv(I_w);

% Simulator inertia
I_s = [0.107 0 0; 0 0.108 0; 0 0 0.116];
I_s_inv = inv(I_s);

% Reaction wheels matrix 

% A = eye(3); % RWs along x, y and z directions

% RWs in pyramidal configuration
A = [-1/sqrt(2)  1/sqrt(2)     0;...
     -1/sqrt(6) -1/sqrt(6) sqrt(2/3);...
      1/sqrt(3)  1/sqrt(3) 1/sqrt(3)];
A_inv = pinv(A);

%% Reaction wheels
% state-space representation
A_rw = [-b/J*eye(3) Kt/J*eye(3); -Ke/L*eye(3) -R/L*eye(3)];
B_rw = [zeros(3); 1/L*eye(3)];
C_rw = [eye(6); -b/J*eye(3) Kt/J*eye(3)];

%---LQR design---
% augmented state-space model for reference tracking
C1_rw = C_rw(1:3,:);
A_aug = [A_rw zeros(6,3); -C1_rw zeros(3,3)];
B_aug = [B_rw; zeros(3)];
C_aug = [C1_rw zeros(3)];

% weighting matrices
q = [10 10 10 1 1 1 100 100 100];
r = [1 1 1];
Qlqr = diag(q);
Rlqr = diag(r);
% LQR gains
G_aug = lqr(A_aug,B_aug,Qlqr,Rlqr);


%% Simulator 
A_sim = [zeros(3) eye(3); zeros(3) zeros(3)];
B_sim = [zeros(3); -J/I_s(1)*A(1,:); -J/I_s(5)*A(2,:); -J/I_s(9)*A(3,:);];
C_sim = eye(6);

%---LQR design---
% weighting matrices
q = [100 100 100 10 10 10];
r = [0.01 0.01 0.01];
Qlqr = diag(q);
Rlqr = diag(r);
% LQR gains
G_sim = lqr(A_sim,B_sim,Qlqr,Rlqr);


%% Sensor: Bosch BNO055 

% sensors sampling time
dt = 1/100; % 100 Hz

% digital low-pass filter cut-off frequency
omega = 250;




