% PID controller

close all

load state_space_model.mat

Glead = tf([50, 25],[30, 50,14]);

leadsys1 = feedback(tf1, Glead);

figure()
step(leadsys1)

figure()
impulse(leadsys1)

figure()
rlocusplot(leadsys1)