function x_k1 = estimator_pflow_dynamics(v_k, p_noise_k, total)
% total:= u_k, n_Qi, Q_L_i, Q_di, v_neighbors, ids, i, Ts
u_k = total(1);
n_Qi = total(2);
Q_L_i = total(3);
Q_di = total(4);
if length(total) == 8
    v_neighbors = total(5);
    ids = total(6);
    i = total(7);
    Ts = total(8);
elseif length(total) == 10
    v_neighbors = total(5:6);
    ids = total(7:8);
    i = total(9);
    Ts = total(10);
end

B = zeros(4,4);
B(1,2) = 1/10;
B(2,3) = 1/10.67;
B(3,4) = 1/9.82;
B = B + B';

d = v_k(2,1);
v_k = v_k(1,1);
% define voltage dynamics
j = 1;
interm = 0;
while j <= length(v_neighbors)
    interm = interm - v_k*v_neighbors(j)*abs(B(i,ids(j))) + v_k^2*abs(B(i,ids(j)));
    j = j + 1;
end

x_k1 = [v_k + Ts*(-v_k - ...
    n_Qi*(Q_L_i + interm - Q_di + d) + u_k) + p_noise_k(1); d + p_noise_k(2)];

end