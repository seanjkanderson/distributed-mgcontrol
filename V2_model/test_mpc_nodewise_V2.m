%test
clc, clear

v_k(1) = 305;
v_k(2) = 296;
v_k(3) = 318;
v_k(4) = 320;

N = 6;
v_ref = 310;
n_Qi = 5;
Q_L_i = 50;
Q_di = 60;
B = [0.8, 0.2, 0.5, 0.9;
     0.2, 0.7, 0.4, 0.2;
     0.5, 0.4, 0.5, 0.6;
     0.9, 0.2, 0.6, 0.7];
Ts = .3;
Q = 2*eye;
W = 10*eye;
R = 1*eye;

v_cache = [];
t = 0;
tic;
for k = 1:30
    disp(k)
    v_k1 = zeros(4,1);
    %%%%%%%%%%%%%%%%%%%%%%
    % node 1
%     disp('-----Node 1-----');
    self_id = 1;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(2)];
    ids = [2];
    mu_k = mpc_run_nodewise_mdl_fast(v_k_i, v_neighbors, ids, ...
                                          self_id,N,v_ref, Q, W, R, t, Ts);
    v_k1(self_id) = pflow_dynamics(v_k_i, mu_k, v_neighbors, v_ref, Ts);              
 
    %%%%%%%%%%%%%%%%%%%%%%
    % node 2
%     disp('-----Node 2-----');
    self_id = 2;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(1); v_k(3)];
    ids = [1; 3];
    
    mu_k = mpc_run_nodewise_mdl_fast2(v_k_i, v_neighbors, ids, ...
                                          self_id,N,v_ref, Q, W, R, t, Ts);
    v_k1(self_id) = pflow_dynamics(v_k_i, mu_k, v_neighbors, v_ref, Ts);
    %%%%%%%%%%%%%%%%%%%%%%
    % node 3
%     disp('-----Node 3-----');
    self_id = 3;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(2); v_k(4)];
    ids = [2; 4];
    mu_k = mpc_run_nodewise_mdl_fast3(v_k_i, v_neighbors, ids, ...
                                          self_id,N,v_ref, Q, W, R, t, Ts);
    v_k1(self_id) = pflow_dynamics(v_k_i, mu_k, v_neighbors, v_ref, Ts);
    %%%%%%%%%%%%%%%%%%%%%%
    % node 4
%     disp('-----Node 4-----');
    self_id = 4;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(3)];
    ids = [3];
    
    mu_k = mpc_run_nodewise_mdl_fast4(v_k_i, v_neighbors, ids, ...
                                          self_id,N,v_ref, Q, W, R, t, Ts);
    v_k1(self_id) = pflow_dynamics(v_k_i, mu_k, v_neighbors, v_ref, Ts);
    %%%%%%%%%%%%%%%%%%%%%%
    v_cache = [v_cache, v_k1];
    v_k = v_k1;
    t = t + .01;
end
toc;
figure(1); plot(v_cache'); title('Voltage restoration'); xlabel('Iteration');
ylabel('Voltage (p.u.)'); legend('DG 1', 'DG 2', 'DG 3', 'DG 4');