% PID controller
% for input 2 output 2 (tf_4)

clear all;
clc;
run model.m; % to load model parameters

% Define transfer function manually so as to avoid floating point errors
H4 = zpk(tf_4);
tol = 0.1;
H4 = minreal(H4, tol); % perform pole zero cancellation
%%
% pid controller
% using pidtool(H1) to select gains
% with response time set to 0.1 sec and
% transient behavior set to 0.8
Kp = 20904.5453;
Ki = 18289.1073;
Kd = 5973.5011;
PID = pid(Kp,Ki,Kd);

% closed-loop
cl_sys_4 = feedback(H4*PID, 1);

% openloop step response % after pole zero cancellation
figure();
step(H4);
title("step response of the openloop system (sys-2) (after pole zero cancellation)");

% step response
t = linspace(0, 10, 100);  % time array
u = zeros(1,length(t));
u(10:end) = 1; % step input
y = lsim(cl_sys_4, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title("step response of the closed-loop system (sys-2)");

% sinusoidal response
t = linspace(0, 10, 100);  % time array
u = sin(130*t);   % sinusoidal input
y = lsim(cl_sys_4, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title("sinusoidal response of the closed-loop system (sys-2)");

% square wave response
t = linspace(0, 10, 100);  % time array
u = square(t);  % square wawe input
y = lsim(cl_sys_4, u, t); % response to input
figure();
plot(t, y, t, u);
title("square wave response of the closed-loop system (sys-2)");

%% Parameters variation
% add some variation in A matrix
% A = A + delta_A    % delta_A = 0.1
Am = sys_ss.A + 0.3;
Bm = B(:,2);
Cm = C(2,:);
Dm = 0;
[num,den] = ss2tf(Am,Bm,Cm,Dm);
H4_var = tf(num,den);
H4_var = zpk(H4_var);
tol = 0.1;
H4_var = minreal(H4_var, tol);

% now use controller defined before for H1_var
cl_sys_4_var = feedback(H4_var*PID, 1);

% comparision after parameter variation
figure();
step(cl_sys_4, cl_sys_4_var);
title("comparision of step response after introducing parameter variation (sys-2)");
legend("before parameter variation", "after parameter variation");

%% noise in the system
% add noise in the feedback
% C = C + delta_C % delta_c = noise = 0.1
Cm = Cm + 0.1;
[num,den] = ss2tf(Am,Bm,Cm,Dm);
H4_noise = tf(num,den);
H4_noise = zpk(H4_noise);
tol = 0.1;
H4_noise = minreal(H4_noise, tol);

% now use controller defined before for H1_var
cl_sys_4_noise = feedback(H4_noise*PID, 1);

% comparision after parameter variation
figure();
step(cl_sys_4, cl_sys_4_noise);
title("comparision of step response after introducing noise (sys-2)");
legend("before adding noise", "after adding noise");
