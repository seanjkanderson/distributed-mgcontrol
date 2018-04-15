% MPC solver for quasi-real-time, node-wise instead of quasi-distributed
function [u_k,v_k] = mpc_run_nodewise_slow(g, vk, v_neighbors, ids, self_id, t)

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

u = sdpvar(N,1);
v = sdpvar(N+1,1);

objective = 0;
constraints = [v(1,1) == vk, 295 <= v(1) <= 325];
% , .95 <= v(1) <= 1.05
for k = 1:N
        objective = objective + ...
                    objective_fn(g, v(k), v_neighbors, u(k), ...
                                v_ref, Q, R);
        constraints = [constraints, v(k+1) == pflow_dynamics(...
                        v(k), u(k), n_Qi, Q_L_i, Q_di, v_neighbors, B,...
                        ids, self_id, Ts), 295 <= v(k+1) <= 325];
%                     , .95 <= v(k+1) <= 1.05
end
objective = objective + objective_fn(g, v(N+1), v_neighbors, u(N), ...
                                v_ref, Q, 0*R);

options = sdpsettings('verbose',0,'solver','fmincon','fmincon.MaxIter',50);

diag = optimize(constraints, objective, options);

if diag.problem == 1
    error('Infeasible');
else
    u_k = double(u(1));
    v_k = double(v);
end
