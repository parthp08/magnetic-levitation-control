% PID controller

close all
load state_space_model.mat

% Define transfer function manually so as to avoid floating point errors
H1 = tf([0.003374, 0, -0.020294],[1, 0, 0.2052, 0, -39.79]);
H2 = tf([0.003374, 0, 0.02163],[1, 0, 0.2052, 0, -39.79]);

KP = 50000;
KI = 2;
KD = 3;

PID = pid(KP, KI, KD);
CL1 = feedback(H1, PID);
CL2 = feedback(H2, PID);

figure()
impulseplot(CL1)

figure()
stepplot(CL1)

% figure()
% impulseplot(CL2)
% 
% figure()
% stepplot(CL2)