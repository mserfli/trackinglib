function ca_kalman_filter_prediction_comparison()
    clc;
    disp('--- CA Model Comparison: KF vs. UDU-KF vs. UDUI (6-State) ---');

    % time increment for prediction
    dt = 0.1;
    % CA model state transition matrix (6x6): [X, VX, AX, Y, VY, AY]
    Phi = blkdiag([1, dt, 0.5*dt^2; 0, 1, dt; 0, 0, 1], [1, dt, 0.5*dt^2; 0, 1, dt; 0, 0, 1]);
    inv_Phi = blkdiag([1, -dt, 0.5*dt^2; 0, 1, -dt; 0, 0, 1], [1, -dt, 0.5*dt^2; 0, 1, -dt; 0, 0, 1]);
    Q_diag_val = [100.0, 100.0] * dt^2;
    % CA model process noise matrix (6x2): [AX, AY] noise
    G = [0.5*dt^2, 0;
               dt, 0;
                1, 0;
                0, 0.5*dt^2;
                0, dt;
                0, 1];
    Q_sys = G * diag(Q_diag_val) * G';

    % initial state and covariance
    x_init = [10; 2; 2; 0.0125; 0.05; 0.1];
    P_init = diag([5.0, 1.0, 1.0, 1.0, 0.1, 1.0]);

    % initialize standard KF & UDU-KF
    x_KF = x_init; P_KF = P_init;
    x_UKF = x_init; [U_UKF, D_UKF] = udu_factorization(P_init);

    % initialize UDU-IF (UDUI)
    Y_init = inv(P_init);
    z_UIF = Y_init * x_init;
    [U_UIF, D_UIF] = udu_factorization(Y_init);
    
    steps=5; % number of prediction steps
    for k = 1:steps
        % --- A. Standard KF ---
        x_KF = Phi * x_KF;
        P_KF = Phi * P_KF * Phi' + Q_sys;

        % --- B. UDU-KF (Thornton) ---
        [x_UKF, U_UKF, D_UKF] = thornton(x_UKF, Phi, U_UKF, D_UKF, G, diag(Q_diag_val));

        % --- C. UDU-IF (UDUI) ---
        % 1. Time propagation: Reduce information (process noise)
        G_tilde = inv_Phi * G;
        for j = 1:2
            g_j = G_tilde(:, j);
            dq_j = Q_diag_val(j);
            % Rank-1 update for Y = (I - K*g')*Y
            [U_UIF, D_UIF, z_UIF] = udui_rank1_noise(U_UIF, D_UIF, z_UIF, g_j, dq_j);
        end
        % 2. State transformation: z = inv(Phi)' * z und Y = inv(Phi)' * Y * inv(Phi)
        z_UIF = inv_Phi' * z_UIF;
        % 3. Covariance transformation of U, D factors via Thornton (using Q=0)
        [~, U_UIF, D_UIF] = thornton(zeros(6,1), inv_Phi', U_UIF, D_UIF, zeros(6,2), zeros(2,2));
    end

    % covariance matrix reconstruction
    P_UKF_res = U_UKF * D_UKF * U_UKF';
    Y_UIF_res = U_UIF * D_UIF * U_UIF';
    P_UIF_res = inv(Y_UIF_res);
    x_UIF_res = Y_UIF_res \ z_UIF;

    fprintf('\state vector x (steps %d):\n', steps);
    fprintf('Idx |    Standard KF    |      UDU-KF       |      UDU-IF      \n');
    for i=1:6, fprintf('%d   | %15.7f | %17.7f | %17.7f\n', i, x_KF(i), x_UKF(i), x_UIF_res(i)); end
    
    fprintf('\nFrobenius norm P (UDU_KF): %e\n', norm(P_KF - P_UKF_res, 'fro'));
    fprintf('\nFrobenius norm P (UDU_IF): %e\n', norm(P_KF - P_UIF_res, 'fro'));
    disp(P_KF);
    disp(P_UIF_res);
end
