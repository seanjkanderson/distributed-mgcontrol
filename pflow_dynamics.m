function con = pflow_dynamics(n_neighbors,v_k, u_k, ...
                                v_neighbors, ids, i)

% define voltage dynamics
j = 1;
interm = 0;
while j <= n_neighbors
    interm = interm + v_k*v_neighbors(j) * abs(B(i,ids(j)));
    j = j + 1;
end

con = v_k + Ts*(-v_k - ...
    n_Qi*(Q_L_i + v_k^2 * abs(B(i,i)) - interm - Q_di) + u_k);

end