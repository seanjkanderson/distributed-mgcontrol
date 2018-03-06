% MPC solver for quasi-real-time, node-wise instead of quasi-distributed
function [u_k] = mpc_run_nodewise(g, vk, v_neighbors, ids, self_id)

n_neighbors = size(v_neighbors,1);

u = sdpvar(N,1);
v = sdpvar(N+1,1);

objective = 0;
constraints = [v(1,1) == vk];
for k = 1:N
        objective = objective + ...
                    objective_fn(n_neighbors, g, v(k), v_neighbors, ...
                                u(k), Q, R, v_ref);
                                % n_neighbors, g, v_k_i, v_neighbors, u,...
                            % Q, R, v_ref
        constraints = [constraints, v(k+1) == pflow_dynamics(n_neighbors,...
                        v(k),u(k), n_Qi, Q_L_i, Q_di, v_neighbors, B,...
                        ids, self_id, Ts), .9 <= v(k) <= 1.1];
end

options = sdpsettings('verbose',0,'solver','IPOPT');

diag = solvesdp(constraints,objective,options);
diagnostics = diag.problem;

% process output
if diagnostics == 1
    error('The problem is infeasible');
end

u_k = double(u(1));

end
    