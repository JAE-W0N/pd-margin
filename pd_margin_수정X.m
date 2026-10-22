clear;
clc;
close all;

%% 1. Satellite parameter
J = 0.08;                 % Moment of inertia [kg*m^2]

%% 2. Satellite plant
s = tf('s');
G = 1/(J*s^2);

disp('Satellite Plant G(s):')
G

%% 3. PD 제어기
Kp = 0.005;
Kd = 0.028;
Tf = 0.02;

C = pid(Kp,0,Kd,Tf);

disp('PD Controller C(s):')
C

%% 4. Reaction Wheel dynamics
tau_rw = 0.05;             % RW time constant [s]

G_rw = 1/(tau_rw*s + 1);

disp('Reaction Wheel G_{rw}(s):')
G_rw

%% 5. Open-loop transfer function
L = C * G_rw * G;

disp('Open-loop Transfer Function L(s):')
L
%% 6. Bode Plot
figure;
bode(L);
grid on;
title('Open-loop Bode Plot');

%% 7. Gain Margin and Phase Margin
figure;
margin(L);
grid on;
title('Gain Margin and Phase Margin');

[Gm, Pm, Wcg, Wcp] = margin(L);

fprintf('\n===== Stability Margins =====\n');

if isinf(Gm)
    fprintf('Gain Margin = Inf\n');
else
    fprintf('Gain Margin = 약 %.1f배\n', Gm);
    fprintf('Gain Margin = %.2f dB\n', 20*log10(Gm));
end

fprintf('Phase Margin = %.2f deg\n', Pm);
fprintf('Phase crossover frequency Wcg = %.4f rad/s\n', Wcg);
fprintf('Gain crossover frequency Wcp = %.4f rad/s\n', Wcp);

%% 8. Closed-loop system
T_closed = feedback(L,1);

%% 8. 10-degree attitude command response
theta_cmd_deg = 10;
theta_cmd = deg2rad(theta_cmd_deg);

t = 0:0.01:10;
[y,t] = step(theta_cmd*T_closed,t);

figure;
plot(t,rad2deg(y),'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Attitude Angle [deg]');
title('Closed-loop Attitude Response to 10 deg Command');
yline(theta_cmd_deg,'--','Reference');

%% 9. Step response information
info = stepinfo(theta_cmd*T_closed);

fprintf('\n===== Step Response (10 deg command) =====\n');
fprintf('Rise Time     = %.4f s\n', info.RiseTime);
fprintf('Settling Time = %.4f s\n', info.SettlingTime);
fprintf('Overshoot     = %.2f %%\n', info.Overshoot);
%% 10. Controller torque command

% Transfer function: reference angle -> controller torque command
T_torque = feedback(C, G_rw*G);

[tau_cmd, t_tau] = step(theta_cmd*T_torque, t);

figure;
plot(t_tau, tau_cmd*1000, 'LineWidth', 1.5);
grid on;

xlabel('Time [s]');
ylabel('Torque Command [mNm]');
title('Reaction Wheel Torque Command');

yline(1.0, '--', 'Torque Mode Limit (+1 mNm)');
yline(-1.0, '--', 'Torque Mode Limit (-1 mNm)');

% Maximum commanded torque
max_tau = max(abs(tau_cmd));

fprintf('\n===== Reaction Wheel Torque Demand =====\n');
fprintf('Maximum Torque Command = %.4f mNm\n', max_tau*1000);