% Reduced order observer
% for input 2 output 2 (tf_4)
% assuming that x1 is directly avialable
% Ref: Modern Control Engineering, OGOTA, page 767-775

clear all;
clc;
run model.m; % to load model parameters

% create SISO system (TF_1)
Am = A;
Bm = B(:,2);
Cm = C(2,:);
Dm = 0;
sys4 = ss(Am,Bm,Cm,Dm);

% FSFB
% look at the openloop eigenvalues of the system
openloop_poles = eig(Am);

% Verify that the system is Controllable
Pc = ctrb(Am,Bm);
rank(Pc)  % should be equal to 4

% Choose desired closed loop pole locations
desired_closed_loop_poles = [-9;-5;-8;-7];
%desired_closed_loop_poles = [-2;-1;-3;-1];  % change these to see different results

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

%%% Reduced order

% Desired observer poles (3 because only 3 estimation)
desired_estimator_poles = [-12;-12;-12];

Aaa = Am(1,1);
Aab = Am(1,2:end);
Aba = Am(2:end,1);
Abb = Am(2:end,2:end);
Ba = Bm(1);
Bb = Bm(2:end);

% characteristic equation
alphas = charpoly(diag(desired_estimator_poles));

% desired characteristic polynomial
phi_e = alphas(1)*(Abb^3) + ...
        alphas(2)*(Abb^2) + ...
        alphas(3)*(Abb) + ...
        alphas(4)*(eye(3));

% Ackermann's equation
G = phi_e * inv([Aab; Aab*Abb; Aab*(Abb^2)]) * [0;0;1];

% Observer - Controller Transfer Function
A_hat = Abb - G*Aab;
B_hat = A_hat*G + Aba - G*Aaa;
F_hat = Bb - G*Ba;

Ka = K(1);
Kb = K(2:end);

A_ = A_hat - F_hat*Kb;
B_ = B_hat - F_hat*(Ka + Kb*G);
C_ = -Kb;
D_ = -(Ka + Kb*G);
red_obsv_4 = ss(A_,B_,C_,D_);

% Transfer function for reduced order observer
[num,den] = ss2tf(A_,B_,C_,D_);
tf_red_4 = -tf(num,den);

% observer-controller with model
cl_red_4 = feedback(tf_4, tf_red_4);
Nbar = 5.563e08/582.1;

% step response
figure();
step(Nbar*cl_red_4);
title('step response with reduced order observer (sys-2)');

% sinusoidal response
t = linspace(0, 10, 100);  % time array
u = sin(130*t);   % sinusoidal input
y = lsim(Nbar*cl_red_4, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title('sinusoidal response with reduced order observer (sys-2)');

