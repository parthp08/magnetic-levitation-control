% Full order observer
% for input 1 output 1 (tf_1)
% http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace#45

clear all;
clc;
run model.m; % to load model parameters

% OCF form 
Ao = ocf1.A;
Bo = ocf1.B;
Co = ocf1.C;
Do = ocf1.D;

% desired estimator poles % 2 to 4 times faster response than system
desired_estimator_poles = [-10;-6;-8;-5];
%desired_estimator_poles = [-5;-2;-3;-1];

% characteristic polynomial coefficients
alphas = charpoly(diag(desired_estimator_poles));

% desired characteristic polynomial
alpha_e = alphas(1)*(Ao^4) + ...
          alphas(2)*(Ao^3) + ...
          alphas(3)*(Ao^2) + ...
          alphas(4)*(Ao) + ...
          alphas(5)*(eye(4));

% observability matrix
Ox = [Co;
      Co*Ao;
      Co*Ao^2;
      Co*Ao^3;];
  
% Ackermann's equation
G = alpha_e * inv(Ox) * [0;0;0;1];

% sys_1
Am = A;
Bm = B(:,1);
Cm = C(1,:);
Dm = 0;
sys_1 = ss(Am,Bm,Cm,Dm);

% Estimator
est_1 = estim(sys_1, G);

%%
% FSFB
% look at the openloop eigenvalues of the system
openloop_poles = eig(Am);

% Verify that the system is Controllable
Pc = ctrb(Am,Bm);
rank(Pc)  % should be equal to 4

% Choose desired closed loop pole locations
%desired_closed_loop_poles = [-4;-3;-2;-1];
desired_closed_loop_poles = [-9;-2;-5;-3];  % change these to see different results

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

% form a new Model with observer
At = [Am-(Bm*K), Bm*K; zeros(size(Am)), A-(G*Cm)]; % Ref: OGATA page 760
Bt = [Bm*Kr; zeros(size(Bm))];
Ct = [Cm, zeros(size(Cm))];
Dt = 0;

% Observer-Controller with model
[num,den] = ss2tf(At,Bt,Ct,Dt);
cl_obsv_1 = tf(num,den);

% step response
figure();
step(cl_obsv_1);
title('step response with full order observer (sys-1)')

% sinusoidal response
t = linspace(0, 10, 100);  % time array
u = sin(130*t);   % sinusoidal input
y = lsim(cl_obsv_1, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
title('sinusoidal response with full order observer (sys-1)');

% Transfer function of Observer-Controller %Ref: OGATA page 761
[num,den] = ss2tf(est_1.A, est_1.B, est_1.C(1,:), est_1.D(1,1));
tf_obsv_1 = tf(num,den);
