% Initialize the mg_simulator
clc, clear, close all
% Choose the sample time/step size
Ts = .01;
% Choose the simulation horizon
Tsim = 7;
% Choose the control horizon
N = 2;
% Choose the voltage reference
v_ref = 310;
%Choose the frequency reference
w_ref = 50;
% Define the costs
Q = 5*eye;
W = 10*eye;
R = 1*eye;

% only for true model system
% tau = 1e-2;

% Take 310V to be the reference voltage for the network
Init = [100; 200; 300; 400];
% Init = zeros(4,1); 

% Define which nodes have access to the reference voltage
% g_vector = [1 1 1 1];
g_vector = zeros(1,4);

% Choose the time (sec) for the secondary controller to start working
Thresh = 1.5;
% Choose the start time for the droop controller to start
droop_enable = .5;

% Define the susceptances for the lossless network (1/Ohm)
B = zeros(4,4);
B(1,2) = 1/10;
B(2,3) = 1/10.67;
B(3,4) = 1/9.82;
mismatch = 0*rand(4,4)*1e-2;
B = B + B' + mismatch;
% disp(B)

% Define the desired reactive power
Q_di = 0*ones(1,4);
% define the reactive power draw at each node
Q_Li = 0*ones(1,4);

% define the v-Q gains
n_Qi = 4.24e-4*ones(1,4);
n_Pi = [6, 3, 2, 1.5]*1e-5;

Ts_noise = .001;
% define the process noise variance
p_noise = 1e5*ones(1,4);
p_noise_est = 1e6;
p_noise_est = {p_noise_est*[1 0;0 5],...
                p_noise_est*[1 0;0 5],...
                p_noise_est*[1 0;0 5],...
                p_noise_est*[1 0;0 5]};
% p_noise_est = {p_noise_est, p_noise_est, p_noise_est, p_noise_est};
% define the measurement noise variance
w_noise = 1e2*ones(4,1)';
w_noise_est = w_noise;
% define initial state
init_state = {[v_ref; 0],...
                [v_ref; 0],...
                [v_ref; 0], ...
                [v_ref; 0]};
dist = [50 100 150 50];

% define the loads at each node
L_load = 100*ones(4,1);
R_load = .1*ones(4,1);