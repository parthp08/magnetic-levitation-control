% openloop analyis of the system

clear all;
clc;
run model.m; % to load model parameters

%% input 1 output 1
H1 = zpk(tf_1);
%H1 = minreal(H1, 0.1); % to cancel-out pole zero

% impulse response
figure();
impulse(H1);
title("impulse response of the openloop system (sys-1)")

% step response
figure();
step(H1);
title("step response of the openloop system (sys-1)")

% Bode plot
figure();
bode(H1);
title("bode plot of the openloop system (sys-1)")

% Root-locus
figure();
rlocus(H1);
title("rootlocus plot of the openloop system (sys-1)")


%% input 2 output 2
H4 = zpk(tf_4);
%H4 = minreal(H4, 0.1); % to cancel-out pole zero

% impulse response
figure();
impulse(H4);
title("impulse response of the openloop system (sys-2)")

% step response
figure();
step(H4);
title("step response of the openloop system (sys-2)")

% Bode plot
figure();
bode(H4);
title("bode plot of the openloop system (sys-2)")

% Root-locus
figure();
rlocus(H4);
title("rootlocus plot of the openloop system (sys-2)")

