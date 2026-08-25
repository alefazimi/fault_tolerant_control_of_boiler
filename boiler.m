clc; clear all; close all;
%% System definition


A = [3.7451e-15 7.6548e-6 0 0; ...
-4.0887e-6 -6.5527e-2 0 0; ...
2.3773e-6 5.9026e-4 -0.1426 0; ...
-8.1593e-14 -5.5355e-2 18.216 0.08333];

eig(A)

B = [0.0015 -0.0015 -6.9678e-12; ...
-5.9548e-5 -9.0316e-5 5.9647e-11; ...
3.4622e-5 5.2512e-5 3.2492e-11; ...
-0.0167 0.0239 -1.0733e-9];

C = [0.05 -0.0484 6.7129 0.05; ...
0 1 0 0];

D = [0 0 0; ...
0 0 0];
F = ones(4,1);

x0 = 0.999*ones(4,1);

%% Reference model definition

A_m = [-0.01 0 0; ...
0 -0.02 0; ...
0 0 -0.03];

B_m = [1 0 0; ...
0 1 0; ...
0 0 1];

C_m = [0 1 0; ...
1 0 0];
D_m = [0 0 0; ...
0 0 0];


%% Controller design

Bound = 1.05e-3;
Gamma = diag([1.002 0.999 1.001]);


%% Reference-model

H = [-0.00264411452837401,  0.276368234354978,  0.00485669922298695; ...
-0.00212093673792562,  0.126828183853013,  0.00297639139790861; ...
-11263.6759461726,     221638.898137783,    1282.33499200163];

G = [0.968 -114.258 -1; ...
1     0        0; ...
0     1        0; ...
0     0        1];

M = [0.322180266848393,  -23.9048793908242,   -0.198387888997022; ...
0.273413353570640,  -16.9472760226837,   -0.136907357417107; ...
1077571.96327408,   -5503793.87571974,   -55561.6496148715];


%% Controller gains

K_D = [0.0691 0 0 0.0691; ...
0 0.0691 0 0; ...
0 0 0.0691 0];
K_I = 2*[eye(3),[1;0;0]];
K_P = 0.1*[eye(3),[1;0;0]];
K_S = 0.01*eye(3);

%% Adaptive second-order sliding-mode parameters

gamma   = 0.0013;
alpha_0 = 8.0314;
alpha_1 = 1.4713;

%% Running simulation #1

dB = zeros(4,3);
dC = zeros(2,4);
um_factor = 0;
fault = 0;

sim simulation.slx

%% Plot - Simulation #1


figure(1)
subplot(1,2,1)
plot(y1,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 1')
legend('y_1','y_{m1}')
xlabel('Time (s)')
subplot(1,2,2)
plot(ey1,'LineWidth',1.1)
grid
title('Output Error 1')
xlabel('Time (s)')

figure(2)
subplot(1,2,1)
plot(y2,'LineWidth',1.1)
grid
xlabel('Time (s)')
title('Boiler and Reference System Output 2')
legend('y_2','y_{m2}')
subplot(1,2,2)
plot(ey2,'LineWidth',1.1)
grid
title('Output Error 2')
xlabel('Time (s)')


%% Reference model states


figure(3)
subplot(3,1,1)
plot(xm1,'LineWidth',1.1)
grid
title('States of the Reference Model')
xlabel('')
ylabel('x_{m1}')

subplot(3,1,2)
plot(xm2,'LineWidth',1.1)
grid
title('')
xlabel('')
ylabel('x_{m2}')

subplot(3,1,3)
plot(xm3,'LineWidth',1.1)
grid
title('')
xlabel('Time(s)')
ylabel('x_{m3}')


%% Linearized boiler states


figure(4)
subplot(4,1,1)
plot(x1,'LineWidth',1.1)
grid
title('States of the Linearized Model')
ylabel('x_1')

subplot(4,1,2)
plot(x2,'LineWidth',1.1)
grid
title('')
ylabel('x_2')

subplot(4,1,3)
plot(x3,'LineWidth',1.1)
grid
title('')
ylabel('x_3')

subplot(4,1,4)
plot(x4,'LineWidth',1.1)
grid
title('')
xlabel('Time(s)')
ylabel('x_4')


%% Auxiliary system states


figure(5)

subplot(4,1,1)
plot(z1,'LineWidth',1.1)
grid
title('Auxiliary System States')
ylabel('z_1')

subplot(4,1,2)
plot(z2,'LineWidth',1.1)
grid
title('')
ylabel('z_2')

subplot(4,1,3)
plot(z3,'LineWidth',1.1)
grid
title('')
ylabel('z_3')

subplot(4,1,4)
plot(z4,'LineWidth',1.1)
grid
title('')
xlabel('Time(s)')
ylabel('z_4')

%% Running simulation #2

um_factor = 1;
fault = 0;

dB = 1.02e-4*[1 2 0; ...
0 0 0; ...
1 -1 0; ...
20 -20 0];

dC = 0.0002*[1 0 0 0;0 0 0 0];

sim simulation.slx


%% Plot - Simulation #2

figure(6)
subplot(2,2,1)
plot(y1,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 1 (in presence of fault)')
legend('y_1','y_{m1}')
xlabel('Time(s)')

subplot(2,2,2)
plot(ey1,'LineWidth',1.1)
grid
title('Output Error 1')
xlabel('Time(s)')

subplot(2,2,3)
plot(y2,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 2 (in presence of fault)')
legend('y_2','y_{m2}')
xlabel('Time (s)')

subplot(2,2,4)
plot(ey2,'LineWidth',1.1)
grid
title('Output Error 2')
xlabel('Time(s)')


%% Running simulation #3

um_factor = 1;
fault = 0;
dB = zeros(4,3);
dC = 0.0002*[1 0 0 0; ...
0 0 0 0];

sim simulation.slx


%% Plot - Simulation #3


figure(7)
subplot(2,2,1)
plot(y1,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 1')
legend('y_1','y_{m1}')
xlabel('Time (s)')

subplot(2,2,2)
plot(ey1,'LineWidth',1.1)
grid
title('Output Error 1')
xlabel('Time(s)')
subplot(2,2,3)
plot(y2,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 2')
legend('y_2','y_{m2}')
xlabel('Time(s)')

subplot(2,2,4)
plot(ey2,'LineWidth',1.1)
grid
title('Output Error 2')
xlabel('Time(s)')

figure(8)
subplot(2,2,1)
plot(y1,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 1')
legend('y_1','y_{m1}')
xlabel('Time (s)')

subplot(2,2,2)
plot(ey1,'LineWidth',1.1)
grid
title('Output Error 1')
xlabel('Time(s)')

subplot(2,2,3)
plot(y2,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 2')
legend('y_2','y_{m2}')
xlabel('Time(s)')

subplot(2,2,4)
plot(ey2,'LineWidth',1.1)
grid
title('Output Error 2')
xlabel('Time (s)')


%% Running simulation #4

um_factor = 1;
fault = 0;
dB = 2.04e-4*[1 2 0; ...
0 0 0; ...
1 -1 0; ...
20 -20 0];

dC = 0.0002*[1 0 0 0;0 0 0 0];

sim simulation.slx

%% Plot - Simulation #4


figure(9)
subplot(2,2,1)
plot(y1,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 1')
legend('y_1','y_{m1}')
xlabel('Time (s)')

subplot(2,2,2)
plot(ey1,'LineWidth',1.1)
grid
title('Output Error 1')
xlabel('Time (s)')
subplot(2,2,3)
plot(y2,'LineWidth',1.1)
grid
title('Boiler and Reference System Output 2')
legend('y_2','y_{m2}')
xlabel('Time (s)')
subplot(2,2,4)
plot(ey2,'LineWidth',1.1)
grid
title('Output Error 2')
xlabel('Time (s)')
