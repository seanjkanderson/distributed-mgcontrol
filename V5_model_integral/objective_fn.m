function cost = objective_fn(v_k_i, v_neighbors, u, u_ref, ...
                                v_ref, Q, W, R)
% define (0 -> N) objective function w/ handle
n_neighbors = size(v_neighbors,1);
e = @(v_k_i,v_k_j,v_ref) (v_k_i(1) - v_k_j);
j = 1;
cost = 0;
while j <= n_neighbors
    cost = cost + ...
        e(v_k_i,v_neighbors(j),v_ref)*Q*e(v_k_i,v_neighbors(j),v_ref)'+ ...
        (v_k_i(1) - v_ref)*W*(v_k_i(1) - v_ref)';
    j = j + 1;
end
cost = cost + (u- u_ref)*R*(u-u_ref);


end