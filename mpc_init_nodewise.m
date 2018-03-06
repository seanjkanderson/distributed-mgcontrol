
% choose the voltage reference
v_ref = 4160*sqrt(2/3);
% define the horizon lengths
% prediction horizon
Lsim = 15;
% control horizon
N = 3;
% sampling time
Ts = 0.3;

% define time invariant costs
Q = eye(1);
R = eye(1);

% define susceptance matrix
B = [];