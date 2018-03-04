%% MPC formulation for DG voltage regulation
clc
clear, close all
set_param('plMicrogridIslandOperation','MaxConsecutiveZCsMsg','none');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tune parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% number of DGs
n = 4;
% number of pinned nodes
r = 1;

% define Laplacian
L = [1 -1 0 0; 
    -1 2 -1 0;
    0 -1 2 -1;
    0 0 -1 1];

% define the sampling rate/delay
eps = .3;

% choose the voltage reference
v_ref = 4160*sqrt(2/3);

% tune the cost function (LQR style)
Q = eye(n);
R = eye(n);

Hu = 5; % control horizon
Hp = 15; % prediction horizon

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End tuning section-- don't edit below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define block matrices for efficient solving
% This is governed by \delta X(k+1) = E(PX*x(k) + PU*U(k))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define PU
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
In = eye(n);
psi_init = [eye(r,r), zeros(r,n-r)]';
 
P_eps = In - eps*L;
full_row = [];

% create the longest row that we might see
for i = Hp-1:-1:0
    full_row = [full_row, P_eps^i];
end

% slice this up for each row that is less than longest (i < Hp-1)
P_U = [];
for i = 1:Hp
    if i <= Hu
        P_U = [P_U; full_row(:,end-i*n+1:end), zeros(n, Hu*n-i*n)];
    else
        temp = zeros(n,n);
        kk = i+1;
        while kk > Hu
            temp = temp + full_row(:,end-kk*n+1+Hu*n:end-(kk-1)*n+Hu*n);
            kk = kk - 1;
        end
        P_U = [P_U; full_row(:,end-i*n+1:end-(i+1)*n+Hu*n), temp];
    end
end
psi_full = kron(psi_init, eye(Hu));
P_U = P_U*psi_full;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define PX
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P_X = [];
for i = 1:Hp
    P_X = [P_X; P_eps^i];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define E
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
e = [];
for i = 1:n
    for j = i+1:n
        eij = zeros(1,n);
        eij(i) = 1;
        eij(j) = -1;
        e = [e; eij];        
    end

end

E = kron(e, eye(Hp));

P_UE = E*P_U;
P_XE = E*P_X;
Q_block = eye(size(P_UE,1));
R_block = eye(size(P_UE,2));

% Get the optimal gain matrix from the least squares solution
pred = eye(r);
for i = 1:Hu-1
    pred = [pred, zeros(r,r)];
end

PPC_tilde = pred * inv(P_UE'*Q_block*P_UE + R_block) * P_UE'*Q_block*P_XE;

%% Error checking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do some basic error checks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~all(size(L) == [n,n])
    error('Dimension mismatch in Laplacian');
end

if ~all(size(psi_full) == [Hu*n, Hu*r])
    error('psi_full is improperly constructed');
end

if ~all(size(P_U) == [Hp*n, Hu*r])
    error('P_U is improperly constructed');
end

if ~all(size(P_X) == [Hp*n, n])
    error('P_X is improperly constructed');
end

if ~all(size(e) == [n*(n-1)/2, n])
    error('e is improperly constructed');
end

if ~all(size(E) == [Hp*n*(n-1)/2, Hp*n])
    error('E is improperly constructed');
end