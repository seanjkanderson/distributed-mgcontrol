% MPC solver for quasi-real-time, node-wise instead of quasi-distributed
function [u_k,x_k] = mpc_run_nodewise(g, v_k_i, v_neighbors, ids, self_id, t)

persistent Controller1

if t == 0
    
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
    R = 0*eye;
    
    yalmip('clear')
    
    u = sdpvar(N,1);
    v = sdpvar(N+1,1);
    v_k_i_var = sdpvar(1,1);
    g_var = sdpvar(1,1);
    self_id_var = sdpvar(1,1);
    v_neighbors_var = sdpvar(size(v_neighbors,1),1);
    ids_var = sdpvar(size(v_neighbors,1),1);

    objective = 0;
    constraints = [v(1) == v_k_i_var, 295 <= v(1) <= 325];
    for k = 1:N
            objective = objective + ...
                        objective_fn(g, v(k), v_neighbors_var, u(k), ...
                                    v_ref, Q, R);
            constraints = [constraints, v(k+1) == pflow_dynamics(...
                            v(k),u(k), n_Qi, Q_L_i, Q_di, v_neighbors_var, B,...
                            ids, self_id, Ts), 295 <= v(k+1) <= 325];
    end
    objective = objective + objective_fn(g, v(N+1), v_neighbors_var, u(N), ...
                                    v_ref, Q, 0*R);

    options = sdpsettings('verbose',0,'solver','fmincon');
    
    % initial state and reference
    Controller1 = optimizer(constraints,objective,options,{g_var, v_k_i_var, ...
                            v_neighbors_var, ids_var, self_id_var},{u(1),v(1:2)});
    
    [u_k, errorcode] = Controller1({g, v_k_i, v_neighbors, ids, self_id});
    
else
    [u_k, errorcode] = Controller1({g, v_k_i, v_neighbors, ids, self_id});
end
if errorcode == 1
    error('Infeasible');
end
x_k = 0;
% disp(u_k{2});
u_k = u_k{1};
end

    