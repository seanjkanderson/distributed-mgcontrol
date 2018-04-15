% MPC solver for quasi-real-time, node-wise instead of quasi-distributed
function u_k = mpc_run_nodewise_mdl(g, vk, v_neighbors, ids, ...
                    self_id, N, v_ref, n_Qi, Q_L_i, Q_di, Ts, Q, R, t)

B = [0 .1 0 0;
    0.1 0 .0937 0;
    0 .0937 0 .1018;
    0 0 0.1018 0];
          
u = sdpvar(N,1);
v = sdpvar(N+1,1);

objective = 0;
constraints = [v(1,1) == vk, 295 <= v(1) <= 325];
for k = 1:N
        objective = objective + ...
                    objective_fn(g, v(k), v_neighbors, u(k), ...
                                v_ref, Q, R);
        constraints = [constraints, v(k+1) == pflow_dynamics(...
                        v(k), u(k), n_Qi, Q_L_i, Q_di, v_neighbors, B,...
                        ids, self_id, Ts), 295 <= v(k+1) <= 325];
end
objective = objective + objective_fn(g, v(N+1), v_neighbors, u(N), ...
                                v_ref, Q, 0*R);

options = sdpsettings('verbose',0,'solver','fmincon','fmincon.MaxIter',50);

diag = optimize(constraints, objective, options);

if diag.problem == 1
    error('Infeasible');
else
    u_k = double(u(1));
    disp('---------------');
    disp(self_id);
    disp(double(v(1)));
    disp(u_k);
end
