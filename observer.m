% Full order observer and reduced order observer

load state_space_model.mat

% NOTE: This code is currently incorrect, I was just playing around with
% the method shown on page 97 of the lecture slides.

%% Design observer gain matrix G1 & G2
% Define the (random) desired estimator poles
ePoles = [-10;-6;-8;-5];

% Get the estimator char. poly. coefficients
eCoeffs = charpoly(diag(ePoles));

% Try using Ackermann's method
A1 = ocf1.A;
C1 = ocf1.C;

Ox = [C1;
      C1*A;
      C1*A^2;
      C1*A^3]
  
obsv(A1,C1)
  
alphae = eCoeffs(1)*A1^4 + eCoeffs(2)*A1^3 + eCoeffs(3)*A1^2 + eCoeffs(4)*A1 + eCoeffs(5)*eye(4);

G11 = alphae * inv(Ox) * [0;0;0;1]
%% Make a regulator for state-feedback and estimator gains

% Note, use the 'reg()' function built in to MATLAB