% ----------------
% data_preload_4RW
%
% Numerical data for air bearing simulation with 4 reaction wheels

% Reaction Wheel and Motor data
Ke = 0.008;    % V/rad/s
Kt = 0.00797;  % N*m/A
L = 25e-6;     % H
R = 0.73;      % Ohm
b = 1.05e-6;   % N*m/rad/s
J = 1.74e-4 + 5.1e-7;    % kg*m^2


I_w = J*eye(4);
I_w_inv = inv(I_w);

% Simulator inertia
I_s = [0.107 0 0; 0 0.108 0; 0 0 0.116];
I_s_inv = inv(I_s);

% Reaction wheels matrix 

% RWs in pyramidal configuration
a = 1/sqrt(3);

A = [-a  a  a -a;...
     -a -a  a  a;...
      a  a  a  a];
A_inv = pinv(A);

%% Reaction wheels
% state-space representation
A_rw = [-b/J*eye(4) Kt/J*eye(4); -Ke/L*eye(4) -R/L*eye(4)];
B_rw = [zeros(4); 1/L*eye(4)];
C_rw = [eye(8); -b/J*eye(4) Kt/J*eye(4)];

%---LQR design---
% augmented state-space model for reference tracking
C1_rw = C_rw(1:4,:);
A_aug = [A_rw zeros(8,4); -C1_rw zeros(4,4)];
B_aug = [B_rw; zeros(4)];
C_aug = [C1_rw zeros(4)];

% weighting matrices
q = [10 10 10 10 1 1 1 1 100 100 100 100];
r = [1 1 1 1];
Qlqr = diag(q);
Rlqr = diag(r);
% LQR gains
G_aug = lqr(A_aug,B_aug,Qlqr,Rlqr);


%% Simulator 
A_sim = [zeros(3) eye(3); zeros(3) zeros(3)];
B_sim = [zeros(3,4); -J/I_s(1)*A(1,:); -J/I_s(5)*A(2,:); -J/I_s(9)*A(3,:);];
C_sim = eye(6);

%---LQR design---
% weighting matrices
q = [10 10 10 1 1 1];
r = [0.1 0.1 0.1 0.1];
Qlqr = diag(q);
Rlqr = diag(r);
% LQR gains
G_sim = lqr(A_sim,B_sim,Qlqr,Rlqr);


%% Sensor: Bosch BNO055 

% sensors sampling time
dt = 1/100; % 100 Hz

% digital low-pass filter cut-off frequency
omega = 250;




