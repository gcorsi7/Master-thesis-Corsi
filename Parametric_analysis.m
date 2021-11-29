% -------------------
% Parametric analysis
%
% Script used for the parametric analysis of both electrical and 
% mechanical power of the system (n is the parameter)
%
% Run it with Simulink opened
% Note: the parametric analysis was done with all sensors disabled
%

n_i = linspace(0.0001,0.5,100);


for i = 1:length(n_i)
    n = n_i(i);
    
    input_preload
    data_preload % can be modified with "data_preload_4RW"
    
    model = gcs;
    outputs = sim(model);
    
    % max power (peak)
    P_mec(i,:) = max(outputs.yout.signals(9).values);
    P_el(i,:) = max(outputs.yout.signals(6).values);
    eff(i) = P_mec(i,1)/P_el(i,1);
    
    % mean power
    tf = find(outputs.tout == 21); % find index of final time tf
    % take only values from t0 to tf
    P_mec_temp = outputs.yout.signals(9).values(1:tf,3);
    P_el_temp = outputs.yout.signals(6).values(1:tf,3);
    % take the mean value
    P_mec_mean(i) = mean(P_mec_temp);
    P_el_mean(i) = mean(P_el_temp);
    mean_eff(i) = P_mec_mean(i)/P_el_mean(i);
end

figure(1)
plot(n_i, P_mec(:,1), 'linewidth', 1.8)
grid on;
title('P_{mec} vs n')
xlabel('n');
ylabel('P_{mec}');

figure(2)
plot(n_i, P_el(:,1), 'linewidth', 1.8)
grid on;
title('P_{el} vs n')
xlabel('n');
ylabel('P_{el}');

figure(3)
plot(n_i, eff, 'linewidth', 1.8)
grid on;
title('eff vs n')
xlabel('n');
ylabel('\eta');

figure(4)
plot(n_i, P_mec_mean, 'linewidth', 1.8)
grid on;
title(' mean P_{mec} vs n')
xlabel('n');
ylabel('$\bar{P_{mec}}$', 'interpreter', 'LaTex');

figure(5)
plot(n_i, P_el_mean, 'linewidth', 1.8)
grid on;
title('mean P_{el} vs n')
xlabel('n');
ylabel('$\bar{P_{el}}$', 'interpreter', 'LaTex');

figure(6)
plot(n_i, mean_eff, 'linewidth', 1.8)
grid on;
title('mean eff vs n')
xlabel('n');
ylabel('$\bar{\eta}$', 'interpreter', 'LaTex');