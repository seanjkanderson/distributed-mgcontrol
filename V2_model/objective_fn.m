function cost = objective_fn(v_k_i, v_neighbors, mu_v, ...
                                v_ref, Q, W, R, Ts)
% define (0 -> N) objective function w/ handle
n_neighbors = size(v_neighbors,1);

% deltaV = @(v_k_i_dv,v_k_j_dv) (v_k_i_dv - v_k_j_dv);
interm = sum(v_neighbors - v_k_i);
%-------define u-------
interm = 0;
j = 1;
while j <= max(size(v_neighbors))
    interm = interm + (v_neighbors(j) - v_k_i);
    j = j + 1;
end
u = (Ts + mu_v)*(interm - (v_k_i - v_ref));
%----------------------

j = 1;
cost = 0;
while j <= n_neighbors
    cost = cost + ...
        (v_neighbors(j)-v_k_i)*Q*(v_neighbors(j)-v_k_i);
    j = j + 1;
end
cost = cost + (v_k_i - v_ref)*W*(v_k_i - v_ref) + u*R*u;
end