function u_ref = find_target(v_ref, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, i)

n_neighbors = size(v_neighbors,1);
v_neighbors(:) = v_ref;
% define voltage dynamics
j = 1;
interm = 0;
while j <= n_neighbors
%     interm = interm + v_k*v_neighbors(j) * abs(B(i,ids(j)));
    interm = interm - v_ref*v_neighbors(j)*abs(B(i,ids(j))) + v_ref^2*abs(B(i,ids(j)));
    j = j + 1;
end

u_ref = -(-v_ref - n_Qi*(Q_L_i + interm - Q_di));
end