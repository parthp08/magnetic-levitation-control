% Full state feedback controller
% for input 2 output 2 (tf_4)

clear all;
clc;
run model.m; % to load model parameters

% create SISO system (TF_1)
Am = A;
Bm = B(:,2);
Cm = C(2,:);
Dm = 0;
sys4 = ss(Am,Bm,Cm,Dm);

% look at the openloop eigenvalues of the system
openloop_poles = eig(Am);

% Verify that the system is Controllable
Pc = ctrb(Am,Bm);
rank(Pc)  % should be equal to 4

% Choose desired closed loop pole locations
%desired_closed_loop_poles = [-4;-3;-2;-1];
%desired_closed_loop_poles = [-9;-2;-5;-3];  % change these to see different results
desired_closed_loop_poles = [-2;-3;-2.5;-3];

% compute full state feedback gain
K = acker(Am,Bm,desired_closed_loop_poles);

% check that the eigenvalues are at locations where we want them to be
A_cl = Am - Bm*K;
eig(A_cl)  % should be same as desired_closed_loop_poles

% create closed loop system
cl_sys4 = ss(A_cl, Bm, Cm, Dm);

% correction for steady state error in the FSFB controller
% solve for Kr  % https://www.youtube.com/watch?v=FXSpHy8LvmY
Kdc = dcgain(cl_sys4);
Kr = 1/Kdc;

% Create scaled input closed loop system
cl_sys_4_scaled = ss(A_cl, Bm*Kr, Cm, Dm);

% step response
t = linspace(0, 10, 100);  % time array
u = zeros(1,length(t));
u(10:end) = 1; % step input
y = lsim(cl_sys_4_scaled, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title('step response of closed loop system(scaled) (sys-2)');

% sinusoidal response
t = linspace(0, 10, 100);  % time array
u = sin(130*t);   % sinusoidal input
y = lsim(cl_sys_4_scaled, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title('sinusoidal response of closed loop system(scaled) (sys-2)');

% square wave response
t = linspace(0, 10, 100);  % time array
u = square(t);  % square wawe input
y = lsim(cl_sys_4_scaled, u, t); % response to input
figure();
plot(t, y, t, u);
title('square wave response of closed loop system(scaled) (sys-2)');

