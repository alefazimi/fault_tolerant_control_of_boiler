%% Corrected boiler.m
clc
clear
close all
warning off

%% System definition %%
% State order used by the linearized model:
% x = [Vwt; p; ar; Vsd]
%
% Input order used by the matrices:
% u = [qf; qs; Q]

A = [ 3.7451e-15    7.6548e-6     0           0;
     -4.0887e-6    -6.5527e-2     0           0;
      2.3773e-6     5.9026e-4    -0.1426      0;
     -8.1593e-14   -5.5355e-2    18.216       0.08333];

B = [ 0.0015       -0.0015       -6.9678e-12;
     -5.9548e-5    -9.0316e-5     5.9647e-11;
      3.4622e-5     5.2512e-5     3.2492e-11;
     -0.0167        0.0239       -1.0733e-9];

C = [ 0.0500       -0.0484        6.7129       0.0500;
      0             1              0           0];

D = zeros(2,3);

% Disturbance/uncertainty channel placeholder.
% Keep it zero for the paper simulations unless your simulation.slx
% explicitly uses a matched disturbance signal.
F = zeros(4,1);

% For the linearized deviation model, nominal operating point is zero.
% If your Simulink integrators use deviation states, use zeros.
x0 = zeros(4,1);

% Nominal physical operating point, only for reference/post-processing.
x_op = [57.5; 8.5; 0.051; 4.8];
u_op = [32.00147798; 32.00147798; 80.40437506e6];
y_op = [1.2089; 1];

% Display open-loop poles for verification.
disp('Open-loop poles of linearized boiler A matrix:')
disp(eig(A))

%% Reference model definition %%
A_m = [-0.01   0      0;
        0     -0.02   0;
        0      0     -0.03];

B_m = [1 0 0;
       0 1 0;
       0 0 1];

C_m = [0 1 0;
       1 0 0];

D_m = zeros(2,3);

%% Controller design %%
% Boundary-layer thickness for tanh(S_dot/Bound).
Bound = 1e-3;

% Switching matrix magnitude.
% The paper requires a negative definite switching matrix in v_sw.
% If your simulation.slx already implements "-Gamma*S", keep Gamma positive.
% If your simulation.slx implements "+Gamma*S", change this line to:
% Gamma = -diag([1,1,1]);
Gamma = diag([1,1,1]);

% Model-following matrices used by your working simulation.slx.
% These preserve your existing working structure.
G = [0.968, -114.258, -1;
     1,       0,       0;
     0,       1,       0;
     0,       0,       1];

H = [-0.00264411452837401,  0.276368234354978,   0.00485669922298695;
     -0.00212093673792562,  0.126828183853013,   0.00297639139790861;
    -11263.6759461726,      221638.898137783,    1282.33499200163];

M = [ 0.322180266848393,   -23.9048793908242,   -0.198387888997022;
      0.273413353570640,   -16.9472760226837,   -0.136907357417107;
      1077571.96327408,   -5503793.87571974,   -55561.6496148715];

% Check paper matching residuals.
E_state  = A*G + B*H - G*A_m;
E_output = C*G - C_m;
E_input  = B*M - G*B_m;

fprintf('\nModel matching residuals:\n');
fprintf('  max|A*G + B*H - G*A_m| = %.3e\n', max(abs(E_state(:))));
fprintf('  max|C*G - C_m|         = %.3e\n', max(abs(E_output(:))));
fprintf('  max|B*M - G*B_m|       = %.3e\n', max(abs(E_input(:))));

% Paper PID sliding-surface matrices.
K_P = [1.3731   0       0       1.3731;
       0        1.3731  0       0;
       0        0       1.3731  0];

K_I = [2.6355   0       0       2.6355;
       0        2.6355  0       0;
       0        0       2.6355  0];

K_D = [0.0691   0       0       0.0691;
       0        0.0691  0       0;
       0        0       0.0691  0];

K_S = 1.4800*eye(3);

% Numerical check for K_D*B.
K_DB = K_D*B;

fprintf('\nK_D*B numerical check:\n');
fprintf('  rank(K_D*B) = %d\n', rank(K_DB));
fprintf('  cond(K_D*B) = %.3e\n', cond(K_DB));

% If K_D*B is rank deficient, add a small regularizing term.
% This prevents numerical problems if simulation.slx uses inv(K_D*B).
% If you want the exact paper matrix only, comment out this block.
if rank(K_DB) < 3
    warning('K_D*B is rank deficient. Adding regularizing term K_D(3,4)=0.0691.');
    K_D(3,4) = 0.0691;
    K_DB = K_D*B;

    fprintf('After regularization:\n');
    fprintf('  rank(K_D*B) = %d\n', rank(K_DB));
    fprintf('  cond(K_D*B) = %.3e\n', cond(K_DB));
end

% Adaptive and switching parameters.
gamma   = 0.0013;
alpha_0 = 8.0314;
alpha_1 = 1.4713;

% Check sliding matrix condition roughly.
fprintf('\nSliding condition check:\n');
fprintf('  1 + max(eig(K_D*B*Gamma)) = %.4f\n', ...
        1 + max(eig(K_DB*Gamma)));

%% Fault definitions %%
% Paper input-fault structure.
dB_base = [1   2    0;
           0   0    0;
           1  -1    0;
           20 -20   0];

% Paper output-fault structure.
dC_base = [ 0.002  -0.001   0       0;
           -0.003   0.002   0       0.001];

% The input fault is scaled to produce approximately 10% variation of B.
dB_fault = 1e-4*dB_base;

% Output fault used directly as in the paper.
dC_fault = dC_base;

% Fault times, if your simulation.slx uses workspace fault-time variables.
input_fault_time  = 80;
output_fault_time = 120;

%% Running simulation #1: Nominal regulator %%
% Paper Part I:
% um = 0, no fault, no uncertainty, no disturbance.
dB = zeros(size(B));
dC = zeros(size(C));
um_factor = 0;
fault = 0;

sim simulation.slx

% Plot simulation #1
figure(1)
subplot(1,2,1)
plot(y1)
grid on
title('Boiler and Reference System Output 1')
legend('y_1','y_{m1}','Location','best')
xlabel('Time (s)')

subplot(1,2,2)
plot(ey1)
grid on
title('Output error 1')
xlabel('Time (s)')

figure(2)
subplot(1,2,1)
plot(y2)
grid on
title('Boiler and Reference System Output 2')
legend('y_2','y_{m2}','Location','best')
xlabel('Time (s)')

subplot(1,2,2)
plot(ey2)
grid on
title('Output error 2')
xlabel('Time (s)')

figure(3)
subplot(3,1,1)
plot(xm1)
grid on
title('States of the reference model')
xlabel('Time (s)')

subplot(3,1,2)
plot(xm2)
grid on
xlabel('Time (s)')

subplot(3,1,3)
plot(xm3)
grid on
xlabel('Time (s)')

figure(4)
subplot(4,1,1)
plot(x1)
grid on
title('States of the linearized boiler')
xlabel('Time (s)')

subplot(4,1,2)
plot(x2)
grid on
xlabel('Time (s)')

subplot(4,1,3)
plot(x3)
grid on
xlabel('Time (s)')

subplot(4,1,4)
plot(x4)
grid on
xlabel('Time (s)')

figure(5)
subplot(4,1,1)
plot(z1)
grid on
title('Auxiliary system states')
xlabel('Time (s)')

subplot(4,1,2)
plot(z2)
grid on
xlabel('Time (s)')

subplot(4,1,3)
plot(z3)
grid on
xlabel('Time (s)')

subplot(4,1,4)
plot(z4)
grid on
xlabel('Time (s)')

%% Running simulation #2: Periodic reference input, no fault %%
% Paper Part II:
% Periodic square signal as the second input of the reference model.
dB = zeros(size(B));
dC = zeros(size(C));
um_factor = 1;
fault = 0;

sim simulation.slx

plot_fault_outputs(y1, ey1, y2, ey2, 6, 'Periodic reference, no fault')

%% Running simulation #3: Input matrix fault %%
% Paper Part III:
% Input fault occurs at t = 80 s.
% If simulation.slx uses the variable "fault" as an enable signal,
% fault = 1 enables the fault logic.
dB = dB_fault;
dC = zeros(size(C));
um_factor = 1;
fault = 1;

sim simulation.slx

plot_fault_outputs(y1, ey1, y2, ey2, 7, 'Input matrix fault')

%% Running simulation #4: Input and output matrix faults %%
% Paper Part IV:
% Input fault remains, output fault occurs at t = 120 s.
dB = dB_fault;
dC = dC_fault;
um_factor = 1;
fault = 1;

sim simulation.slx

plot_fault_outputs(y1, ey1, y2, ey2, 8, 'Input and output matrix faults')

%% Local plotting function %%
function plot_fault_outputs(y1, ey1, y2, ey2, figNo, mainTitle)

    figure(figNo)

    subplot(2,2,1)
    plot(y1)
    grid on
    title([mainTitle ': Output 1'])
    legend('y_1','y_{m1}','Location','best')
    xlabel('Time (s)')

    subplot(2,2,2)
    plot(ey1)
    grid on
    title('Output error 1')
    xlabel('Time (s)')

    subplot(2,2,3)
    plot(y2)
    grid on
    title([mainTitle ': Output 2'])
    legend('y_2','y_{m2}','Location','best')
    xlabel('Time (s)')

    subplot(2,2,4)
    plot(ey2)
    grid on
    title('Output error 2')
    xlabel('Time (s)')

end