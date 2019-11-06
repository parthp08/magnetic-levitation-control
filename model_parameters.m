% Define model parameters

%global a b c d N yc m g

a = 1.65;
b = 6.20;
c = 2.69;
d = 4.20;
N = 4;
%yc = 0.120;     %m
yc = 12.0;
%m = 0.120;      %kg
m = 120.0;
g = 9.81;       %m/s^2

% Collect parameters
parameters = [a,b,c,d,N,yc,m,g];

% Define operating points
%y1o = 0.02;     %m
y1o = 2.0;
%y2o = -0.02;    %m
y2o = -2.0;

u1o = a*(y1o + b)^N * (c/(yc + y2o - y1o + d)^N + m*g);     %A
u2o = a*(-y2o + b)^N * (-c/(yc + y2o - y1o + d)^N + m*g);   %A

ya = (y1o + b);
yb = (-y2o + b);
yd = yc + y2o - y1o + d;

%% Define our linearized state space model

Am = [0, 0, 1, 0;... 
     0, 0, 0, 1;...
     (-1/m)*(4*u1o/(a*ya^(N+1)) + 4*c/yd^(N+1)), (1/m)*(4*c/yd^(N+1)), 0, 0;...
     (1/m)*(4*c/yd^(N+1)), (1/m)*(4*u2o/(a*yb^(N+1)) - 4*c/yd^(N+1)), 0, 0];
 
Bm = [0, 0;...
     0, 0;...
     1/(m*a*ya^N), 0;...
     0, 1/(m*a*yb^N)];
 
 Cm = [eye(4,2), zeros(4,2)];
 
 Dm = zeros(4,2);

 %charpoly(Am);
 
%% Build state equation, transfer matrix, OCF, CCF, JCF realizations

 state_sys = ss(Am, Bm, Cm, Dm);
 
 tf_sys = tf(state_sys);
 
 ocf_sys = canon(state_sys, 'companion');
 ccf_sys = ocf_sys.';
 [eig_mat, jcf_sys] = jordan(Am);