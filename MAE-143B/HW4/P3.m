%MAE 143B H3 P3

clc; clear; close all;

m =1.1; 
l = 0.9;
g = 9.81;
b = 0.7;
J = m*l^2;

theta_d = pi / 6;

Kp = 75; %75
Kd = 20;  %20

num_PD = [Kd Kp];
den_PD = [J (b+Kd) (m *g *l + Kp)];

sys_PD = tf(num_PD, den_PD);

[y_PD, t_PD] = step(theta_d*sys_PD, 3);
plot(t_PD,y_PD, 'b', 'LineWidth', 1.5); hold on
title('PD Controller Step Response');
ylabel('\theta(t) [rad]');
xlabel('Time[s]')
grid on;

info_PD = stepinfo(theta_d * sys_PD);
disp('---PD Controller Performance---');
disp(info_PD);

xLimits = xlim;
yLimits = ylim;

xText = xLimits(2) - 0.35 * range(xLimits);
yTop = yLimits(2) - 0.05 * range(yLimits);

text(xText, yTop, sprintf('Rise Time: %.3f s', info_PD.RiseTime), 'FontSize', 10);
text(xText, yTop - 0.06 * range(yLimits), sprintf('Settling Time: %.3f s', info_PD.SettlingTime), 'FontSize', 10);
text(xText, yTop - 0.12 * range(yLimits), sprintf('Overshoot: %.1f%%', info_PD.Overshoot), 'FontSize', 10);

saveas(gcf,'stepR_PD.png');

figure;
pzmap(sys_PD);
hold on; grid on;
title('Closed-Loop Poles with Spec Region');
xlabel('Re'); ylabel('Im');
 
zeta_spec = 0.4;
ts_spec = 1.5;
wn_spec = 4.5;
theta_zeta = acos(zeta_spec);
r = linspace(0, 20, 200);
plot(-r*cos(theta_zeta), r*sin(theta_zeta), 'r--');
plot(-r*cos(theta_zeta), -r*sin(theta_zeta), 'r--');
xline(-4/ts_spec, 'g--', 'LineWidth', 1.5);
xline(-wn_spec, 'b--', 'LineWidth', 1.5);
legend('Poles', 'ζ = 0.4', 'Settling Time = 1.5s', 'ω_n = 4.5');


saveas(gcf,'s_plane.png'); 

Ki = 120;  

s = tf('s');
G = 1 / (J*s^2 + b*s + m*g*l);
D_pid = (Kd*s^2 + Kp*s + Ki) / s;
T_pid = feedback(D_pid * G, 1);

figure;
[y_PID, t_PID] = step(theta_d * T_pid, 3);
plot(t_PID, y_PID, 'b', 'LineWidth', 1.5); hold on;
title('PID Controller Step Response');
ylabel('\theta(t) [rad]');
xlabel('Time [s]');
grid on;
 
info_PID = stepinfo(theta_d * T_pid);
disp('--- PID Controller Performance ---');
disp(info_PID);
 
xLimits = xlim;
yLimits = ylim;

xText = xLimits(2) - 0.35 * range(xLimits);  
yTop = yLimits(2) - 0.05 * range(yLimits);   

text(xText, yTop, sprintf('Rise Time: %.3f s', info_PID.RiseTime), 'FontSize', 10);
text(xText, yTop - 0.06 * range(yLimits), sprintf('Settling Time: %.3f s', info_PID.SettlingTime), 'FontSize', 10);
text(xText, yTop - 0.12 * range(yLimits), sprintf('Overshoot: %.1f%%', info_PID.Overshoot), 'FontSize', 10);

saveas(gcf,'stepR_PID.png');

m_new = 1.1 + 1.5;
J_new = m_new * l^2;
mgl_new = m_new * g * l;

theta_d = pi/6;

s = tf('s');
G_disturbed = 1 / (J_new * s^2 + b * s + mgl_new);

D_pd = Kd * s + Kp;
T_pd_disturbed = feedback(D_pd * G_disturbed, 1);

D_pid = (Kd * s^2 + Kp * s + Ki) / s;
T_pid_disturbed = feedback(D_pid * G_disturbed, 1);

figure;
step(theta_d * T_pd_disturbed, 3); hold on;
step(theta_d * T_pid_disturbed, 3);
legend('PD (with ball)', 'PID (with ball)');
title('Response with Unmodeled Disturbance (Ball Added)');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
grid on;

saveas(gcf,'stepR_PD/PID_Dist.png');
