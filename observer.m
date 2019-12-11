% Full order observer and reduced order observer

close all
load state_space_model.mat

% NOTE: This code is currently in progress, playing around with
% the method shown on page 97 of the lecture slides.

%% Design estimator gain matrix G1 & G2

% Define the (random) desired estimator poles
ePoles = [-10;-6;-8;-5];

% Get the estimator char. poly. coefficients
% These coefficients describe the desired transient behaviour
eCoeffs = charpoly(diag(ePoles));

% Define SISO system for magnet 1 in ocf form
A1 = ocf1.A;
B1 = ocf1.B;
C1 = ocf1.C;


G10 = flip(eCoeffs(2:5)).' + A1(:,4)

% Try different approach using Ackermann's method

% Construct observability matric
Ox = [C1;
      C1*A1;
      C1*A1^2;
      C1*A1^3];
  
% Build desired matrix polynomial
ae = eCoeffs(1) * (A1^4) +...
     eCoeffs(2) * (A1^3) +...
     eCoeffs(3) * (A1^2) +...
     eCoeffs(4) * A1     +...
     eCoeffs(5) * eye(4);

% Use Ackmermann's equation
G11 = ae * inv(Ox) * [0;0;0;1]
G12 = ae * (Ox\[0;0;0;1])
%% Make an estimator for state-feedback control

est1 = estim(sys1, G12)

figure()
impulse(est1)

figure()
step(est1)

% Construct a regulator using the FSFB matrix K and estimator gain matrix G 
% This regulator my not be necessary but I stumbled upon it during my
% research and thought it would be useful to leave here
%reg1 = reg(sys1, K1, G12);
%obs1 = feedback(sys1, reg1)

