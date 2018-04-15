% MPC solver for quasi-real-time, node-wise instead of quasi-distributed
function u_k = mpc_run_nodewise_mdl_fast3(v_k_i, v_neighbors, ids, ...
                    self_id, N, v_ref, Q, W, R, t, Ts)

persistent Controller1

if t == 0

    B = zeros(4,4);
    B(1,2) = 1/10;
    B(2,3) = 1/10.67;
    B(3,4) = 1/9.82;
    B = B + B';
    
    yalmip('clear')
    
    mu_v = sdpvar(N,1);
    v = sdpvar(N+1,1);
    v_k_i_var = sdpvar(1,1);
    self_id_var = sdpvar(1,1);
    v_neighbors_var = sdpvar(size(v_neighbors,1),1);
    ids_var = sdpvar(size(v_neighbors,1),1);

    objective = 0;
    constraints = [v(1) == v_k_i_var, v(1) >= 0];
    for k = 1:N
            objective = objective + ...
                        objective_fn(v(k), v_neighbors_var, mu_v(k), ...
                                    v_ref, Q,W, R, Ts);
            constraints = [constraints, ...
                            v(k+1) == pflow_dynamics(v(k),mu_v(k),v_neighbors,v_ref,Ts), ...
                            v(k+1) >= 0];
    end
    objective = objective + objective_fn(v(N+1), v_neighbors_var, mu_v(N), ...
                                    v_ref, Q, W, 0*R, Ts);

    options = sdpsettings('verbose',0);
    
    % initial state and reference
    Controller1 = optimizer(constraints,objective,options,{v_k_i_var, ...
                            v_neighbors_var, ids_var, self_id_var},{mu_v(1),v});
    
    [mu_k, errorcode] = Controller1({v_k_i, v_neighbors, ids, self_id});
    
else
    [mu_k, errorcode] = Controller1({v_k_i, v_neighbors, ids, self_id});
end

if errorcode == 1
    error('Infeasible');
end
% disp(mu_k{2});
u_k = mu_k{1};
% transform to input voltage
% interm = 0;
% j = 1;
% while j <= max(size(v_neighbors))
%     interm = interm + (v_neighbors(j) - v_k_i);
%     j = j + 1;
% end
% 
% u_k = (Ts + mu_k{1})*(interm - (v_k_i - v_ref));

end

    