% Motor Simulation Script
%
% Author: Paul Sampson
%
% Description: Simulates a DC motor in Simulink.

Ra = 1;
Kt = 0.5;
Ke = 0.5;
J = 0.05;
b = 0.5;
T = 6;
K = 8;
sigma = 0.2;

open_system('motorSimQuantized')
motor = sim('motorSimQuantized', T);

figure;
subplot(1,2,1);
plot(motor.inputs(1));
subplot(1,2,2);
plot(motor.outputs);