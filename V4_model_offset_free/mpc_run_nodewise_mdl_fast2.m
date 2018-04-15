% MPC solver for quasi-real-time, node-wise instead of quasi-distributed
function u_k = mpc_run_nodewise_mdl_fast2(v_k_i, v_neighbors, ids, ...
                    self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, W, R, t)
% disp('start');                
% disp([ids, self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, W, R, t]);
persistent Controller1

if t == 0

    % Define the susceptances for the lossless network (1/Ohm)
    B = zeros(4,4);
    B(1,2) = 1/10;
    B(2,3) = 1/10.67;
    B(3,4) = 1/9.82;
    B = B + B';
    
    yalmip('clear')
    
    u = sdpvar(N,1);
    v = sdpvar(N+1,2);
    v_k_i_var = sdpvar(1,2);
    self_id_var = sdpvar(1,1);
    v_neighbors_var = sdpvar(size(v_neighbors,1),1);
    ids_var = sdpvar(size(v_neighbors,1),1);

    objective = 0;
    constraints = [v(1,:) == v_k_i_var];
    u_target = find_target(v_ref, n_Qi, Q_L_i, Q_di,...
                                v_neighbors_var, B, ids, self_id);
    disp(u_target)
    for k = 1:N
        objective = objective + ...
            objective_fn(v(k,:), v_neighbors_var, u(k), u_target, ...
            v_ref, Q, W, R);
        constraints = [constraints, v(k+1,:) == pflow_dynamics(...
            v(k,:),u(k), n_Qi, Q_L_i, Q_di, v_neighbors_var, B,...
            ids, self_id, Ts)', v(k+1) < 1.05*v_ref];
    end
    objective = objective + objective_fn(v(N+1,:), v_neighbors_var, u(N), ...
                                    u_target, v_ref, Q, W, 0*R);

    options = sdpsettings('verbose',0,'solver','fmincon');
    
    % initial state and reference
    Controller1 = optimizer(constraints,objective,options,{v_k_i_var, ...
                            v_neighbors_var, ids_var, self_id_var},{u(1),v(1:2)});
    
    [u_k, errorcode] = Controller1({v_k_i, v_neighbors, ids, self_id});
    
else
    [u_k, errorcode] = Controller1({v_k_i, v_neighbors, ids, self_id});
end
if errorcode == 1
    error('Infeasible');
end
% disp(u_k{2});
disp('');
u_k = u_k{1};

end

    