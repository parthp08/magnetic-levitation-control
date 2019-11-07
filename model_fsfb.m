%%% Magnetic Levitation System
%%% using Full State Feedback Controller

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
     (((4*c)/(m*yd^5))),                        ((4*u2_0)/(m*a*yb^5))+((-4*c)/(m*yd^5)),  0,   0];

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

%% Full state feedback controller design (For TF_1)

% create SISO system (TF_1)
Am = A;
Bm = B(:,1);
Cm = C(1,:);
Dm = 0;
sys_1 = ss(Am,Bm,Cm,Dm);

% look at the openloop eigenvalues of the system
openloop_poles = eig(A)

% Verify that the system is Controllable
Pc = ctrb(A,B);
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
cl_sys_1 = ss(A_cl, Bm, Cm, Dm);

% open loop step response
figure();
step(sys_1);

% closed loop step response 
figure();
step(cl_sys_1);

% solve for Kr
Kdc = dcgain(cl_sys_1);
Kr = 1/Kdc;

% Create scaled input closed loop system
cl_sys_1_scaled = ss(A_cl, Bm*Kr, Cm, Dm);
figure();
step(cl_sys_1_scaled);
