B%% Satellite 1-Axis Attitude Control
% PD control + Bode plot + Stability Margins

clear;
clc;
close all;

%% 1. 위성 파라미터
J_sat = 0.08;  % 단위 : kg⋅m2

%% 2. 위성 Plant
s = tf('s');
G_sat = 1/(J_sat*s^2);

disp('Satellite Plant G_sat(s):')
G_sat

%% 3. PD 제어기
Kp = 0.005;
Kd = 0.028;
T_d_filter = 0.02;

C_d = (Kd*s)/(T_d_filter*s + 1);

% 피드백 제어기
C_pd_fb = Kp + C_d;

disp('Feedback PD Controller C_{pd,fb}(s):')
C_pd_fb

%% 4. 리액션 휠 동특성 반영
% RW time constant [s](추정값) / RW 시험 후 값 수정
T_rw = 0.05; 

G_rw = 1/(T_rw*s + 1);

disp('Reaction Wheel G_{rw}(s):')
G_rw

%% 5. Open-loop 전달 함수
G_plant = G_rw * G_sat;

L_open = C_pd_fb * G_plant;

disp('Open-loop Transfer Function L_{open}(s):')
L_open

%% 6. Bode Plot
figure;
bode(L_open);
grid on;
title('Open-loop Bode Plot');

%% 7. Gain Margin / Phase Margin
figure;
margin(L_open);
grid on;
title('Gain Margin and Phase Margin');

[Gm, Pm, Wcg, Wcp] = margin(L_open);

fprintf('\n===== Stability Margins =====\n');

if isinf(Gm)
    fprintf('Gain Margin = Inf\n');
else
    fprintf('Gain Margin = 약 %.1f배\n', Gm);
    fprintf('Gain Margin = %.2f dB\n', 20*log10(Gm));
end

fprintf('Phase Margin = %.2f deg\n', Pm);
fprintf('Phase crossover frequency Wcg = %.2f rad/s\n', Wcg);
fprintf('Gain crossover frequency Wcp = %.2f rad/s\n', Wcp);

%% 8. Closed-loop 시스템
T_ref_to_angle = minreal((Kp * G_plant) / (1 + C_pd_fb * G_plant));

%% 9. 10도 명령에 대한 반응
theta_ref_deg = 10;
theta_ref = deg2rad(theta_ref_deg);

t_sim = 0:0.01:30;
[theta_response,t_sim] = step(theta_ref*T_ref_to_angle,t_sim);

figure;
plot(t_sim,rad2deg(theta_response),'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Attitude Angle [deg]');
title('Closed-loop Attitude Response to 10 deg Command');
yline(theta_ref_deg,'--','Reference');
ylim([0 11]);

%% 10. Step response information
step_info = stepinfo(theta_ref*T_ref_to_angle);

fprintf('\n===== Step Response (10 deg command) =====\n');
fprintf('Rise Time     = %.4f s\n', step_info.RiseTime);
fprintf('Settling Time = %.4f s\n', step_info.SettlingTime);
fprintf('Overshoot     = %.2f %%\n', step_info.Overshoot);

%% 11. 제어기 토크 명령

% Transfer function: reference angle -> controller torque command
T_ref_to_torque = minreal(Kp / (1 + C_pd_fb * G_plant));

[tau_cmd, t_torque] = step(theta_ref*T_ref_to_torque, t_sim);

figure;
plot(t_torque, tau_cmd*1000, 'LineWidth', 1.5);
grid on;

xlabel('Time [s]');
ylabel('Torque Command [mNm]');
title('Reaction Wheel Torque Command');

yline(1.0, '--', 'Torque Mode Limit (+1 mNm)');
yline(-1.0, '--', 'Torque Mode Limit (-1 mNm)');

% Maximum commanded torque
tau_cmd_max = max(abs(tau_cmd));

fprintf('\n===== Reaction Wheel Torque Demand =====\n');
fprintf('Maximum Torque Command = %.4f mNm\n', tau_cmd_max*1000);