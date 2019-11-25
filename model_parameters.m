close all

% Define model parameters

a = 1.65;
b = 6.20;
c = 2.69;
d = 4.20;
N = 4;
yc = 0.120;     %m
m = 0.120;      %kg
g = 9.81;       %m/s^2

% Define operating points
y1o = 0.02;     %m
y2o = -0.02;    %m
y12o = yc + y2o - y1o; %m

% I spoke to Dr. Khorasani about the units and we figured out that this
% isn't in Amperes but instead in terms of some units of 'counts' which
% have a linear conversion to Amperes. He said to just work with it for now

% u1o = a*(y1o + b)^N * (4*c/(yc + y2o - y1o + d)^(N+1) + m*g);     %A
% u2o = a*(y2o + b)^N * (4*c/(yc + y2o - y1o + d)^(N+1) + m*g);   %A

u1o = a*(y1o + b)^4 * (4*c/(y12o + d)^5 + m*g);     %A
u2o = a*(-y2o + b)^4 * (-4*c/(y12o + d)^5 + m*g);   %A

% Dummy values to simplify equations
% ya = (y1o + b);
% yb = (-y2o + b);
% yd = yc + y2o - y1o + d;

k1 = 4*u1o/(a*(y1o + b)^5);
k2 = 4*u2o/(a*(-y2o + b)^5);
k12 = 4*c/((y12o + d)^5);
ku1 = 1/(a*(y1o + b)^4);
ku2 = 1/(a*(-y2o + b)^4);

%% Define linearized state space model

A = [0,                  0,            1,   0;...
     0,                  0,            0,   1;...
     -(k1 + k12)/m,      k12/m,        0,   0;...
     k12/m,              (k2 - k12)/m, 0,   0];
 
B = [0,            0;...
     0,            0;...
     ku1/m,        0;...
     0,            ku2/m];
 
 C = [1, 0, 0, 0;
      0, 1, 0, 0];
 
 D = 0;

%% Build state equation, transfer matrix, OCF, CCF, JCF realizations

% Original State and output equation
ss_sys = ss(A, B, C, D);
 
% Transfer matrices and SISO transfer functions
tf_sys = tf(ss_sys);
tf1 = tf_sys(1,1);
tf2 = tf_sys(2,2);
 
%Retrieve OCF, CCF, JCF for each SISO system
ocf1 = canon(tf1, 'companion');
ccf1 = ocf1.';
[sim1, jcf1] = jordan(ocf1.A);

ocf2 = canon(tf2, 'companion');
ccf2 = ocf2.';
[sim2, jcf2] = jordan(ocf2.A);

%% Save parameters

save('state_space_model.mat')
