%test
clc, clear
% v_k = zeros(4,1);
% v_k(1) = .96;
% v_k(2) = .97;
% v_k(3) = 1.02;
% v_k(4) = 1.03;
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
Q = 10*eye;
R = 30*eye;

v_cache = [];
t = 0;
tic;
for k = 1:40
    disp(k)
    v_k1 = zeros(4,1);
    %%%%%%%%%%%%%%%%%%%%%%
    % node 1
%     disp('-----Node 1-----');
    g = 0;
    self_id = 1;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(2)];
    ids = [2];
    [u_k, x_k] = mpc_run_nodewise(g, v_k_i, v_neighbors, ids, self_id, t);
%     v_k1(self_id) = x(2);
    v_k1(self_id) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts);                       
 
    %%%%%%%%%%%%%%%%%%%%%%
    % node 2
%     disp('-----Node 2-----');
    g = 0;
    self_id = 2;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(1); v_k(3)];
    ids = [1; 3];
    
    [u_k, x] = mpc_run_nodewise2(g, v_k_i, v_neighbors, ids, self_id, t);
%     v_k1(self_id) = x(2);
    v_k1(self_id) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts);
    %%%%%%%%%%%%%%%%%%%%%%
    % node 3
%     disp('-----Node 3-----');
    g = 0;
    self_id = 3;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(2); v_k(4)];
    ids = [2; 4];
    
    [u_k, x] = mpc_run_nodewise3(g, v_k_i, v_neighbors, ids, self_id, t);
%     v_k1(self_id) = x(2);
    v_k1(self_id) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts);
    %%%%%%%%%%%%%%%%%%%%%%
    % node 4
%     disp('-----Node 4-----');
    g = 0;
    self_id = 4;
    v_k_i = v_k(self_id);
    v_neighbors = [v_k(3)];
    ids = [3];
    
    [u_k, x] = mpc_run_nodewise4(g, v_k_i, v_neighbors, ids, self_id, t);
%     v_k1(self_id) = x(2);
    v_k1(self_id) = pflow_dynamics(v_k_i, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, self_id, Ts);
    %%%%%%%%%%%%%%%%%%%%%%
    v_cache = [v_cache, v_k1];
    v_k = v_k1;
%     disp(v_k1);
    t = t + .01;
end
toc;
figure(1); plot(v_cache'); title('Voltage restoration'); xlabel('Iteration');
ylabel('Voltage (p.u.)'); legend('DG 1', 'DG 2', 'DG 3', 'DG 4');