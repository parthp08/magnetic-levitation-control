% PID controller
% for input 1 output 1 (tf_1)

clear all;
clc;
run model.m; % to load model parameters

% Define transfer function manually so as to avoid floating point errors
%H1 = tf([0.003374, 0, -0.02093],[1, 0, 0.2108, 0, -39.79]);
H1 = zpk(tf_1);
tol = 0.1;
H1 = minreal(H1, tol); % perform pole zero cancellation

% pid controller
% using pidtool(H1) to select gains
% with response time set to 0.1 sec and
% transient behavior set to 0.8
Kp = 20255.1833;
Ki = 17720.9892;
Kd = 5787.9451;
PID = pid(Kp,Ki,Kd);

% closed-loop
cl_sys_1 = feedback(H1*PID, 1);

% openloop step response % after pole zero cancellation
figure();
step(H1);
title("step response of the openloop system (sys-1) (after pole zero cancellation)");

% step response
t = linspace(0, 10, 100);  % time array
u = zeros(1,length(t));
u(10:end) = 1; % step input
y = lsim(cl_sys_1, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title("step response of the closed-loop system (sys-1)");

% sinusoidal response
t = linspace(0, 10, 100);  % time array
u = sin(130*t);   % sinusoidal input
y = lsim(cl_sys_1, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title("sinusoidal response of the closed-loop system (sys-1)");

% square wave response
t = linspace(0, 10, 100);  % time array
u = square(t);  % square wawe input
y = lsim(cl_sys_1, u, t); % response to input
figure();
plot(t, y, t, u);
title("square wave response of the closed-loop system (sys-1)");

%% Parameters variation
% add some variation in A matrix
% A = A + delta_A    % delta_A = 0.1
Am = sys_ss.A + 0.3;
Bm = B(:,1);
Cm = C(1,:);
Dm = 0;
[num,den] = ss2tf(Am,Bm,Cm,Dm);
H1_var = tf(num,den);
H1_var = zpk(H1_var);
tol = 0.1;
H1_var = minreal(H1_var, tol);

% now use controller defined before for H1_var
cl_sys_1_var = feedback(H1_var*PID, 1);

% comparision after parameter variation
figure();
step(cl_sys_1, cl_sys_1_var);
title("comparision of step response after introducing parameter variation (sys-1)");
legend("before parameter variation", "after parameter variation");

%% noise in the system
% add noise in the feedback
% C = C + delta_C % delta_c = noise = 0.1
Cm = Cm + 0.1;
[num,den] = ss2tf(Am,Bm,Cm,Dm);
H1_noise = tf(num,den);
H1_noise = zpk(H1_noise);
tol = 0.1;
H1_noise = minreal(H1_noise, tol);

% now use controller defined before for H1_var
cl_sys_1_noise = feedback(H1_noise*PID, 1);

% comparision after parameter variation
figure();
step(cl_sys_1, cl_sys_1_noise);
title("comparision of step response after introducing noise (sys-1)");
legend("before adding noise", "after adding noise");

