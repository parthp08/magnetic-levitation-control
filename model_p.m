%%% Magnetic Levitation System

close all

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
     -(((4*u1_0)/(m*a*ya^5))+((4*c)/(m*yd^5))), ((4*c)/(m*yd^5)),                          0,   0;
     (((4*c)/(m*yd^5))),                        ((4*u2_0)/(m*a*yb^5))+((-4*c)/(m*yd^5)),   0,   0];

B = [0,             0;
     0,             0;
     1/(m*a*ya^4),  0;
     0,             1/(m*a*yb^4)];

C = [1, 0, 0, 0;
     0, 1, 0, 0];

D = 0;

% MIMO State-Space Model
sys_ss = ss(A,B,C,D);

% observable canonical form
% ???????
% OCF = canon(sys_ss,'companion'); % ???????

% controllable canonical form
% ???????

% jordan canonical form
% ???????

% Transfer Function Matrix
sys_tf = tf(sys_ss);

% SISO models
tf_1 = sys_tf(1,1);     % input:u1   optput:y1 
%tf_2 = sys_tf(1,2);    % input:u2   optput:y1
%tf_3 = sys_tf(2,1);    % input:u1   optput:y2
tf_4 = sys_tf(2,2);     % input:u2   optput:y2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    TF_1    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% openloop system analysis
% poles_1 = pole(tf_1);
% zeros_1 = zero(tf_1);
% 
% figure();
% impulseplot(tf_1);
% 
% figure();
% stepplot(tf_1);
% 
% figure();
% bodeplot(tf_1);
% 
% figure();
% rlocusplot(tf_1);

%% closed loop system analysis
% Controller Design: PID Controller
% Define Design Specifications ==> ???????
Kp = -1770;
Ki = 5;
Kd = 1560;
PID = pid(Kp,Ki,Kd);
cl_sys1 = feedback(tf_1, PID); % closed loop

% impulse response
figure();
impulseplot(cl_sys1);

% step response
opt = stepDataOptions('StepAmplitude', 1);
figure();
stepplot(cl_sys1, opt);

% Bode plot for closed loop system
figure();
bodeplot(cl_sys1);

% Root locus plot for closed loop system
figure();
rlocusplot(cl_sys1); % notice pole zero cancelation and one dominant pole pair in left half plane

% sinusoidal response
t = linspace(0, 10, 100);  % time array
u = sin(130*t);   % sinusoidal input
y = lsim(cl_sys1, u, t); % response to input
figure();
plot(t, y, t, u); % plotting both input and output
%figure();
%plot(t, y);

% square wave response
t = linspace(0, 10, 100);  % time array
u = square(t);  % square wawe input
y = lsim(cl_sys1, u, t); % response to input
figure();
plot(t, y, t, u);
%figure();
%plot(t, y);

%% introducing noise and parameter variations or uncertainity in the system
% https://www.mathworks.com/help/robust/gs/modeling-uncertainty.html
% ???????

