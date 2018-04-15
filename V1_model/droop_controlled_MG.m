%% Droop-controlled microgrid model

%test without secondary control
w_n = 50;
v_n = 230;

%% Inverter parameters
Vdc = 600;

P_star = 2000;
Q_star = 6000;

% LCL Filter;
L_f = 1.35e-2; % Henrys
r_f = 0.1; % Ohms

L_c = 0.35e-3; %Henrys
r_Lc = 0.03; % Ohms

C_f = 50e-6;

f_s = 8000; %Hz

%% Power Controller
w_c = 31.41; % filter crossover freq
m_p = 9.4e-5; % P_control
n_q = 1.3e-3; % Q control

%% Voltage Controller
% PI gains for voltage controller
K_pv = 0.05;
K_iv = 390;
F = .75; % Proportional gain on current

%% Current Controller
% PI gains for current controller
K_pc = 10.5;
K_ic = 16e3;

%% for Cascaded system

% VSI characteristics
Vdc = 600; %total Vdc bus sum of isolated series dc voltage sources
ncells = 3; %number of cascaded series connected half bridge cells
% this creates (2*ncells+1) levels between -Vdc and +Vdc
config = 1; %power module configuration (switched=1, averaged=2)

% Modulator
fsw = 10000; %carrier wave switching frequency (Hz)
fac = 50; %reference wave frequency (Hz)
Tdt = 0.00*1/fsw; %deadtime (s)

% Three-phase load
Rg = 10; %grid resistance (ohms)
Lg = 0.01; %grid inductance (Henries)

%% Run Model

sim mg_model_cascaded.slx