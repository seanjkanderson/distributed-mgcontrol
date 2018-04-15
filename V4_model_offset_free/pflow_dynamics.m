function con = pflow_dynamics(v_k, u_k, n_Qi, Q_L_i, Q_di,...
                                v_neighbors, B, ids, i, Ts)
d = v_k(2);
v_k = v_k(1);
n_neighbors = size(v_neighbors,1);
% define voltage dynamics
j = 1;
interm = 0;
while j <= n_neighbors
%     interm = interm + v_k*v_neighbors(j) * abs(B(i,ids(j)));
    interm = interm - v_k*v_neighbors(j)*abs(B(i,ids(j))) + v_k^2*abs(B(i,ids(j)));
    j = j + 1;
end

% 
% con = v_k + Ts*(-v_k - ...
%     n_Qi*(Q_L_i + v_k^2 * abs(B(i,i)) - interm - Q_di) + u_k);
con = [v_k + Ts*(-v_k - ...
    n_Qi*(Q_L_i + interm - Q_di) + u_k) + d; d];

end