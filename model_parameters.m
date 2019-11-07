% Define model parameters

%global a b c d N yc m g

a = 1.65;
b = 6.20;
c = 2.69;
d = 4.20;
N = 4;
yc = 0.120;     %m
m = 0.120;      %kg
g = 9.81;       %m/s^2

% Collect parameters
parameters = [a,b,c,d,N,yc,m,g];

% Define operating points
y1o = 0.02;     %m
y2o = -0.02;    %m

% I spoke to Dr. Khorasani about the units and we figured out that this
% isn't in Amperes but instead in terms of some units of 'counts' which
% have a linear conversion to Amperes. He said to just work with it for now
u1o = a*(y1o + b)^N * (c/(yc + y2o - y1o + d)^N + m*g);     %A
u2o = a*(-y2o + b)^N * (-c/(yc + y2o - y1o + d)^N + m*g);   %A

ya = (y1o + b);
yb = (-y2o + b);
yd = yc + y2o - y1o + d;

%% Define our linearized state space model

Am = [0,                                             0,                                         1, 0;... 
      0,                                             0,                                         0, 1;...
      (-1/m)*(4*u1o/(a*ya^(N+1)) + 4*c/yd^(N+1)),    (1/m)*(4*c/yd^(N+1)),                      0, 0;...
      (1/m)*(4*c/yd^(N+1)),                          (1/m)*(4*u2o/(a*yb^(N+1)) - 4*c/yd^(N+1)), 0, 0];
 
Bm = [0,            0;...
      0,            0;...
      1/(m*a*ya^N), 0;...
      0,            1/(m*a*yb^N)];
 
 Cm = [eye(4,2), zeros(4,2)];
 
 Dm = zeros(4,2);

%% Build state equation, transfer matrix, OCF, CCF, JCF realizations

% Original State and output equation
state_sys = ss(Am, Bm, Cm, Dm);
 
% Transfer matrices and SISO transfer functions
tf_sys = tf(state_sys);
tf1 = tf_sys(1,1);
tf2 = tf_sys(2,2);
 
% Retrieve OCF, CCF, JCF for each SISO system
ocf1 = canon(tf1, 'companion');
ccf1 = ocf1.';
[sim_mat1, jcf1] = jordan(ocf1.A);

ocf2 = canon(tf2, 'companion');
ccf2 = ocf2.';
[sim_mat2, jcf2] = jordan(ocf2.A);

%% PID Controller

% Look into Zeigler-Nichols method