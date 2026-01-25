function [U, D, z] = udui_rank1_noise(U, D, z, g, dq)
    % implements the rank-1 update for UDU-IF (UDUI)
    % Y_plus = (I - K*g')*Y
    n = length(z);
    f = U' * g;
    v = diag(D) .* f;
    alpha = f' * v + 1.0/dq;
    K = (U * v) / alpha;
    
    % state update
    z = z - K * (g' * z);
    
    % U,D factor update (Bierman downdate logic)
    % uses the symmetry of (I-Kg')Y(I-Kg')'
    % alpha_prev starts at 1/dq
    alpha_prev = 1.0/dq;
    for j = 1:n
        alpha_curr = alpha_prev + f(j)*v(j);
        D_prev = D(j,j);
        D(j,j) = D_prev * alpha_prev / alpha_curr;
        gamma = f(j) / alpha_prev;
        alpha_prev = alpha_curr;
        for i = 1:j-1
            U_old = U(i,j);
            U(i,j) = U_old - gamma * K_vec(i);
            K_vec(i) = K_vec(i) + v(j) * U_old;
        end
        K_vec(j) = v(j);
    end
end
