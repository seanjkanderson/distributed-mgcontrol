%test
clc, clear

v_k = zeros(4,2);
v_k(1,:) = [305, 50];
v_k(2,:) = [296, 30];
v_k(3,:) = [318, 15];
v_k(4,:) = [320, 0];

N = 2;
v_ref = 310;
n_Qi = 5;
Q_L_i = 50;
Q_di = 60;
B = zeros(4,4);
B(1,2) = 1/10;
B(2,3) = 1/10.67;
B(3,4) = 1/9.82;
B = B + B';
Ts = .3;
Q = 10*eye;
W = 5*eye;
R = .000*eye;

v_cache = [];
t = 0;
tic;
for k = 1:30
    disp(k)
    v_k1 = zeros(4,2);
    %%%%%%%%%%%%%%%%%%%%%%
    % node 1
    disp('-----Node 1-----');
    self_id = 1;
    v_k_i = v_k(self_id,:);
    v_neighbors = [v_k(2,1)];
    ids = [2];
    disp([v_k_i, v_neighbors, ids, ...
                    self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, W, R, t]);
    u_k = mpc_run_nodewise_mdl_fast(v_k_i, v_neighbors, ids, ...
                    self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, W, R, t);
    v_k1(self_id,:) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts);              
 
    %%%%%%%%%%%%%%%%%%%%%%
    % node 2
    disp('-----Node 2-----');
    self_id = 2;
    v_k_i = v_k(self_id,:);
    v_neighbors = [v_k(1,1); v_k(3,1)];
    ids = [1; 3];
    
    u_k = mpc_run_nodewise_mdl_fast2(v_k_i, v_neighbors, ids, ...
                    self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, W, R, t);
    v_k1(self_id,:) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts);    
    %%%%%%%%%%%%%%%%%%%%%%
    % node 3
    disp('-----Node 3-----');
    self_id = 3;
    v_k_i = v_k(self_id,:);
    v_neighbors = [v_k(2,1); v_k(4,1)];
    ids = [2; 4];
    u_k = mpc_run_nodewise_mdl_fast3(v_k_i, v_neighbors, ids, ...
                    self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, W, R, t);
    v_k1(self_id,:) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts); 
    %%%%%%%%%%%%%%%%%%%%%%
    % node 4
    disp('-----Node 4-----');
    self_id = 4;
    v_k_i = v_k(self_id,:);
    v_neighbors = [v_k(3,1)];
    ids = [3];
    
    u_k = mpc_run_nodewise_mdl_fast4(v_k_i, v_neighbors, ids, ...
                    self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, W, R, t);
    v_k1(self_id,:) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts); 
    %%%%%%%%%%%%%%%%%%%%%%
    v_cache = [v_cache, v_k1(:,1)];
    v_k = v_k1;
    t = t + .01;
end
toc;
figure(1); plot(v_cache'); title('Voltage restoration'); xlabel('Iteration');
ylabel('Voltage (p.u.)'); legend('DG 1', 'DG 2', 'DG 3', 'DG 4');