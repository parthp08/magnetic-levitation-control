% Reduced order observer
% for input 1 output 1 (tf_1)
% assuming that x1 is directly avialable
% Ref: Modern Control Engineering, OGOTA, page 767-775

clear all;
clc;
run model.m; % to load model parameters

% create SISO system (TF_1)
Am = A;
Bm = B(:,1);
Cm = C(1,:);
Dm = 0;
sys1 = ss(Am,Bm,Cm,Dm);

% FSFB
% look at the openloop eigenvalues of the system
openloop_poles = eig(Am);

% Verify that the system is Controllable
Pc = ctrb(Am,Bm);
rank(Pc)  % should be equal to 4

% Choose desired closed loop pole locations
%desired_closed_loop_poles = [-4;-3;-2;-1];
desired_closed_loop_poles = [-4;-2;-5;-3];  % change these to see different results

% compute full state feedback gain
K = acker(Am,Bm,desired_closed_loop_poles);

% check that the eigenvalues are at locations where we want them to be
A_cl = Am - Bm*K;
eig(A_cl)  % should be same as desired_closed_loop_poles

% create closed loop system
cl_sys1 = ss(A_cl, Bm, Cm, Dm);

% correction for steady state error in the FSFB controller
% solve for Kr  % https://www.youtube.com/watch?v=FXSpHy8LvmY
Kdc = dcgain(cl_sys1);
Kr = 1/Kdc;

% Create scaled input closed loop system
cl_sys_1_scaled = ss(A_cl, Bm*Kr, Cm, Dm);

%%% Reduced order

% Desired observer poles (3 because only 3 estimation)
desired_estimator_poles = [-20;-15;-18];

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
red_obsv_1 = ss(A_,B_,C_,D_);

% Transfer function for reduced order observer
[num,den] = ss2tf(A_,B_,C_,D_);
tf_red_1 = -tf(num,den);

% observer-controller with model
cl_red_1 = feedback(tf_1, tf_red_1);
Nbar = 6.479e05/2.594e07;

% step response
figure();
step(cl_red_1*Nbar);
title('step response with reduced order observer (sys-1)');

% sinusoidal response
t = linspace(0, 10, 100);  % time array
u = sin(130*t);   % sinusoidal input
y = lsim(cl_red_1*Nbar, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title('sinusoidal response with reduced order observer (sys-1)');
