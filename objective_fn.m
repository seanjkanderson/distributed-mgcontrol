function cost = objective_fn(n_neighbors, g, v_k_i, v_neighbors, u)
% define (0 -> N) objective function w/ handle

e = @(g,v_k_i,v_k_j, v_ref) (v_k_i - v_k_j) + g*(v_k_i - v_ref);

j = 1;
cost = 0;
while j <= n_neighbors
    cost = e(g,v_k_i,v_neighbors(j),v_ref)*Q*e(g,v_k_i,v_neighbors(j),v_ref) + u*R*u;
    j = j + 1;
end

end