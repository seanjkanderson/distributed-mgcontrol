function x_k1 = estimator_pflow_dynamics(v_k, p_noise_k, total)
% total:= u_k, n_Qi, Q_L_i, Q_di, v_neighbors, ids, i, Ts

u_k = total(1);
n_Qi = total(2);
Q_L_i = total(3);
Q_di = total(4);
ids = total(5);
i = total(6);
Ts = total(7);

B = zeros(4,4);
B(1,2) = 1/10;
B(2,3) = 1/10.67;
B(3,4) = 1/9.82;
B = B + B';


v_neighbors = v_k(2);
v_k = v_k(1);
% define voltage dynamics
interm = - v_k*v_neighbors(1)*abs(B(i,ids(1))) + v_k^2*abs(B(i,ids(1)));
    

% x_k1 = [v_k + Ts*(-v_k - ...
%     n_Qi*(Q_L_i + interm - Q_di) + u_k) + d + p_noise_k(1); d + p_noise_k(2)];
x_k1 = [v_k + Ts*(-v_k - ...
    n_Qi*(Q_L_i + interm - Q_di + p_noise_k(1)) + u_k);...
        v_neighbors(1) + p_noise_k(2)];

end