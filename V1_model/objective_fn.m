function cost = objective_fn(g, v_k_i, v_neighbors, u, ...
                                v_ref, Q, R)
% define (0 -> N) objective function w/ handle
n_neighbors = size(v_neighbors,1);

e = @(g,v_k_i,v_k_j,v_ref) (v_k_i - v_k_j) + g*(v_k_i - v_ref);
W = 10;
j = 1;
cost = 0;
while j <= n_neighbors
    cost = cost + ...
        e(g,v_k_i,v_neighbors(j),v_ref)*Q*e(g,v_k_i,v_neighbors(j),v_ref)+ ...
        (v_k_i - v_ref)*W*(v_k_i - v_ref);
    j = j + 1;
end
cost = cost + u*R*u;

end