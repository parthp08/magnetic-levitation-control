%% parameters
a = 1.65;
b = 6.2;
c = 2.69;
d = 4.2;
N = 4;
m = 0.12; % Kg
g = 9.81; % m/s/s

%% operating conditions
y1_0 = 0.02; % m
y2_0 = -0.02; % m
yc = 0.12; % m
y12_0 = yc + y2_0 - y1_0; % m
u1_0 = a*(y1_0+b)^4 * (c/(y12_0+d)^4 + m*g); % A
u2_0 = a*(-y2_0+b)^4 * (-c/(y12_0+d)^4 + m*g); % A

%% Linearized State-Space Model around operating point
yd = y12_0 + d;
ya = y1_0 + b;
yb = -y2_0 + b;
A = [0,                                         0,                                         1,   0;
     0,                                         0,                                         0,   1;
     -(((4*u1_0)/(m*a*ya^5))+((4*c)/(m*yd^5))),   ((4*c)/(m*yd^5)),                          0,   0;
     (((4*c)/(m*yd^5))),                        ((4*u2_0)/(m*a*yb^5))-((4*c)/(m*yd^5)),  0,   0];

B = [0,             0;
     0,             0;
     1/(m*a*ya^4),  0;
     0,             1/(m*a*yb^4)];

C = [1, 0, 0, 0;
     0, 1, 0, 0];

D = 0;

% MIMO State-Space Model
sys_ss = ss(A,B,C,D);

% Transfer Function Matrix
sys_tf = tf(sys_ss);

% SISO models
tf_1 = sys_tf(1,1);     % input:u1   optput:y1 
%tf_2 = sys_tf(1,2);    % input:u2   optput:y1
%tf_3 = sys_tf(2,1);    % input:u1   optput:y2
tf_4 = sys_tf(2,2);     % input:u2   optput:y2

%Retrieve OCF, CCF, JCF for each SISO system
ocf1 = canon(tf_1, 'companion');
ccf1 = ocf1.';
[sim1, jcf1] = jordan(ocf1.A);

ocf4 = canon(tf_4, 'companion');
ccf4 = ocf4.';
[sim4, jcf4] = jordan(ocf4.A);

save('state_space_model.mat')
