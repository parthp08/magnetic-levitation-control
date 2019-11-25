% system model from lab manual 
% just to verify our model

% LTI system on page 63-65

% parameters
a = 1.65;
b = 6.2;
c = 2.69;
d = 4.2;
N = 4;
m = 0.12; % Kg
g = 9.81; % m/s/s

% operating conditions
y1_0 = 0.02; % m
y2_0 = -0.02; % m
yc = 0.12; % m
y12_0 = yc + y2_0 - y1_0; % m
u1_0 = a*(y1_0+b)^4 * (c/(y12_0+d)^4 + m*g); % A
u2_0 = a*(-y2_0+b)^4 * (-c/(y12_0+d)^4 + m*g); % A

k1 = 4*u1_0 / a*(y1_0+b)^5;
k2 = 4*u2_0 / a*(-y2_0+b)^5;
k12 = 4*c / (y12_0+d)^5;
ku11 = 1 / a*(y1_0+b)^4;
ku22 = 1 / a*(-y2_0+b)^4;

A = [0 0 1 0;
    0 0 0 1;
    -(k1+k12)/m k12/m 0 0;
    k12/m (k2-k12)/m 0 0];

B = [0 0;
    0 0;
    ku11/m 0;
    0 ku22/m];

C = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

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

% PID controller design (For TF_1)
Kp = 50;
Ki = 0;
Kd = 2;
PID = pid(Kp,Ki,Kd);
cl_sys1 = feedback(tf_1, PID); % closed loop

%opt = stepDataOptions('StepAmplitude', 1);
figure();
stepplot(tf_1)%, opt);
figure();
stepplot(cl_sys1)%, opt);


figure();
impulseplot(cl_sys1)

figure();
rlocusplot(tf_1)

figure();
rlocusplot(cl_sys1)
