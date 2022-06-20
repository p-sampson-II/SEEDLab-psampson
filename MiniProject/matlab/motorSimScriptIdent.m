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

K = 10;
sigma = 0.2;

open_system('motorSimIdent')
motor = sim('motorSimIdent', T);

figure;
subplot(1,2,1);
plot(motor.Velocity);
subplot(1,2,2);
plot(motor.Position);