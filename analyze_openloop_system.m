% Analyze open-loop system properties
% Created for project modularity

% Copy of work completed by Parth in 'model_p.m'

close all
load state_space_model.mat

%%%%%%%%%%%%%%%%%%%%%%%%%    SISO System 1    %%%%%%%%%%%%%%%%%%%%%%%%%%

poles1 = pole(tf1);
zeros1 = zero(tf1);

figure();
impulseplot(tf1);
title('System 1: Open-loop Impulse Response');

figure();
stepplot(tf1);
title('System 1: Open-loop Step Response');

figure();
bodeplot(tf1);
title('System 1: Open-loop Bode Plot');

figure();
rlocusplot(tf1);
title('System 1: Open-loop Root Locus');


%%%%%%%%%%%%%%%%%%%%%%%%%    SISO System 1    %%%%%%%%%%%%%%%%%%%%%%%%%%

poles2 = pole(tf2);
zeros2 = zero(tf2);

figure();
impulseplot(tf2);
title('System 2: Open-loop Impulse Response');

figure();
stepplot(tf2);
title('System 2: Open-loop Step Response');

figure();
bodeplot(tf2);
title('System 2: Open-loop Bode Plot');

figure();
rlocusplot(tf2);
title('System 2: Open-loop Root Locus');
