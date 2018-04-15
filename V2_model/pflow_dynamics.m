function con = pflow_dynamics(v_k, mu_v, v_neighbors, v_ref, Ts)

% define voltage dynamics
interm = 0;
j = 1;
while j <= max(size(v_neighbors))
    interm = interm + (v_neighbors(j) - v_k);
    j = j + 1;
end

con = v_k + (Ts + mu_v)*(interm - (v_k - v_ref));
end