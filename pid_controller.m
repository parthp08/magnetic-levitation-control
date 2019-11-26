% PID controller
% use PID controller toolbox

close all
load state_space_model.mat

% Define transfer function manually so as to avoid floating point errors
H1 = tf([0.003374, 0, -0.020294],[1, 0, 0.2052, 0, -39.79]);

figure();
opt = stepDataOptions('StepAmplitude', 1);
stepplot(H1,opt); % openloop stepplot

% using pidcontroller toolbox to select gains
%pidtool(H1);
% using PID controller with parallel config
% setting response time to 0.0001 sec
% and setting transient behaviour to 0.72
%print(PID); % output controller from the pid toolbox

% from the output of the pid toolbox
KP = 2.6e+10;
KI = 2.89e+13;
KD = 5.86e+06;
PID = pid(KP, KI, KD);

% closed loop system
CL1 = feedback(H1, PID);

figure();
opt = stepDataOptions('StepAmplitude', 1);
stepplot(CL1, opt); % closedloop stepplot

figure();
impulseplot(CL1);

figure();
rlocusplot(H1);

figure();
rlocusplot(CL1);
