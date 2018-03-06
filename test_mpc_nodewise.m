%test
clc, clear

vk_3 = 226/230;
vk_4 = 232/230;
v_neighbors = [229; 232]/230;
ids = [2; 4];
self_id = 3;
N = 3;
v_ref = 1;
n_Qi = 1;
Q_L_i = .5;
Q_di = 0;
B = rand(4,4);
Ts = .3;
Q = 10*eye;
R = .1*eye;

cache1 = [];
cache2 = [];
cache3 = [];
cache4 = [];

for k = 1:20
    %%%%%%%%%%%%%%%%%%%%%%
    % node 3 has v_ref
    g = 1;
    [u_k_3] = mpc_run_nodewise(g, vk_3, v_neighbors, ids, self_id, N,...
                v_ref, n_Qi, Q_L_i, Q_di, B, Ts, Q, R);
    v_k1_3 = pflow_dynamics(size(ids,1),vk_3, u_k_3, n_Qi, Q_L_i, Q_di, ...
                v_neighbors, B, ids, self_id, Ts);
    cache1 = [cache1; u_k_3, v_k1_3];
    %%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%
    % node 4
    g = 0;
    [u_k_4] = mpc_run_nodewise(g, vk_4, vk_3, ids, self_id, N,...
                v_ref, n_Qi, Q_L_i, Q_di, B, Ts, Q, R);
    v_k1_4 = pflow_dynamics(size(ids,1),vk_4, u_k_4, n_Qi, Q_L_i, Q_di, ...
                v_neighbors, B, ids, self_id, Ts);
    cache4 = [cache4; u_k_4, v_k1_4];
    
    %%%%%%%%%%%%%%%%%%%%%%
    
    vk_3 = v_k1_3;
    vk_4 = v_k1_4;
end



states = [cache1(:,2), cache4(:,2)]