% Initialize the mg_simulator
clc, clear, close all
% Choose the sample time/step size
Ts = .01;
% Choose the simulation horizon
Tsim = 20;
% Choose the control horizon
N = 3;
% Choose the reference voltage
v_ref = 310;
% Define the costs
Q = 10*eye;
W = 1*eye;
R = .1*eye;

% Take 310V to be the reference voltage for the network
% Init = [305; 297; 315; 320];
Init = 310*ones(4,1);

% Define which nodes have access to the reference voltage
g_vector = [1 1 1 1];

% Choose the time (sec) for the secondary controller to start working
Thresh = 21;
% Choose the start time for the droop controller to start
droop_enable = .1;

% Define the susceptances for the lossless network (1/Ohm)
B = zeros(4,4);
B(1,2) = 1/10;
B(2,3) = 1/10.67;
B(3,4) = 1/9.82;
B = B + B';

% Define the desired reactive power
Q_d1 = -.1;
Q_d2 = Q_d1;
Q_d3 = Q_d1;
Q_d4 = Q_d1;

% define the v-Q gains
n_Q1 = .01;
n_Q2 = n_Q1;
n_Q3 = n_Q2;
n_Q4 = n_Q3;

% define the f-P gains
n_P1 = .1;
n_P2 = n_P1;
n_P3 = n_P2;
n_P4 = n_P3;
