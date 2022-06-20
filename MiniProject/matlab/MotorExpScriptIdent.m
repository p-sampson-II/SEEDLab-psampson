% Motor Identification Script
%
% Author: Paul Sampson
%
% Description: Identifies a first-order approximation of a motor from data

Ra = 1;
Kt = 0.5;
Ke = 0.5;
J = 0.05;
b = 0.5;
T = 6;

K = 114.4795944;
sigma = 0.1147802808;

open_system('motorExpIdent')
motor = sim('motorExpIdent', T);

figure;
subplot(1,2,1);
plot(motor.Velocity);
subplot(1,2,2);
plot(motor.Position);