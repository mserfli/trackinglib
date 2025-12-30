function cv_kalman_filter_prediction_comparision()
% =========================================================================
% VERGLEICH PRAEDIKTION: KF vs. IF vs. UDU-KF vs. UDU-IF
% Fokus: Reines Arbeiten auf U- und D-Faktoren
% Modell: 2D Constant Velocity (CV)
% =========================================================================
    clc;
    disp('--- 2D CV Modell: Praediktion ausschließlich über UDU-Faktoren ---');

    % 1. PARAMETER
    T = 1.0; 
    q = 0.1;
    PHI_1D = [1, T; 0, 1];
    PHI = blkdiag(PHI_1D, PHI_1D);
    INV_PHI = blkdiag([1, -T; 0, 1], [1, -T; 0, 1]); % Für UDU-IF benötigt

    Q_1D = q * [T^3/3, T^2/2; T^2/2, T];
    Q = blkdiag(Q_1D, Q_1D);
    [UQ, DQ] = udu_factorization(Q); % Prozessrausch-Faktoren (fest)

    % 2. INITIALISIERUNG
    x_init = [10; 2; 0; 0];
    P_init = [ 5.01025, 0.105,   0,       0;
               0.105,   1.1,     0,       0;
               0,       0,       1.00125, 0.015;
               0,       0,       0.015,   0.2 ];

    % Filter-Zustaende (nur Faktoren!)
    % Standard KF (Referenz)
    x_KF = x_init; P_KF = P_init;

    % UDU-KF (Thornton)
    x_UKF = x_init;
    [U_UKF, D_UKF] = udu_factorization(P_init);

    % UDU-IF (D'Souza/Zanetti)
    % Initialisierung z = Y*x und Y = U*D*U'
    Y_init = inv(P_init);
    z_UIF = Y_init * x_init;
    [U_UIF, D_UIF] = udu_factorization(Y_init);

    % =====================================================================
    % 2 PRAEDIKTIONSSCHRITTE
    % =====================================================================
    for k = 1:2
        % --- A. REFERENZ: Standard Kalman Filter ---
        x_KF = PHI * x_KF;
        P_KF = PHI * P_KF * PHI' + Q;

        % --- B. UDU-KF: Zeitpropagation (Thornton) ---
        x_UKF = PHI * x_UKF;
        [U_UKF, D_UKF] = thornton_update(U_UKF, D_UKF, PHI, UQ, DQ);

        % --- C. UDU-IF: Zeitpropagation (D'Souza/Zanetti) ---
        % 1. Prozessrauschen einrechnen (Rank-1 Updates auf Information)
        G_bar = INV_PHI * UQ; % G_k = I, da Q direkt 4x4
        for i = 1:size(UQ, 2)
            h = G_bar(:, i)';
            r = 1.0 / DQ(i, i); % D_Q = inv(Delta_Q) [cite: 1, 8]
            [U_UIF, D_UIF, K_gain] = carlson_update(U_UIF, D_UIF, h, r);
            z_UIF = (eye(4) - K_gain * h) * z_UIF; % [cite: 1, 9]
        end
        % 2. Transformation durch Transition
        % z = PHI^-T * z [cite: 1, 9]
        z_UIF = INV_PHI' * z_UIF;
        % U, D Update für PHI^-T (MWGS ohne Rauschen)
        [U_UIF, D_UIF] = thornton_update(U_UIF, D_UIF, INV_PHI', zeros(4), zeros(4,4));
    end

    % =====================================================================
    % AUSGABE (Matrix-Rekonstruktion nur am Ende zur Anzeige)
    % =====================================================================
    P_UKF_res = U_UKF * D_UKF * U_UKF';
    Y_UIF_res = U_UIF * D_UIF * U_UIF';
    x_UIF_res = Y_UIF_res \ z_UIF;
    P_UIF_res = inv(Y_UIF_res);

    disp(' ');
    fprintf('Zustandsvektor x nach 2 Schritten:\n');
    fprintf('Index |      KF      |    UDU-KF    |    UDU-IF    \n');
    fprintf('-----------------------------------------------------\n');
    for i = 1:4
        fprintf('  %d   | %12.7f | %12.7f | %12.7f \n', ...
            i, x_KF(i), x_UKF(i), x_UIF_res(i));
    end

    disp(' ');
    disp('Kovarianzmatrix P (Rekonstruiert aus Faktoren):');
    disp('Standard KF:'); disp(P_KF);
    disp('UDU-KF (Thornton):'); disp(P_UKF_res);
    disp('UDU-IF (D''Souza):'); disp(P_UIF_res);

    fprintf('Max. Abweichung UDU-KF vs KF: %e\n', norm(P_KF - P_UKF_res, 'fro'));
    fprintf('Max. Abweichung UDU-IF vs KF: %e\n', norm(P_KF - P_UIF_res, 'fro'));
end

% --- HILFSFUNKTIONEN ---

function [U, D] = udu_factorization(P)
    % Initial-Zerlegung
    n = size(P, 1); U = eye(n); D = zeros(n);
    for j = n:-1:1
        D(j,j) = P(j,j) - U(j, j+1:n).^2 * diag(D(j+1:n, j+1:n));
        for i = 1:j-1
            U(i,j) = (P(i,j) - U(i, j+1:n) * diag(D(j+1:n, j+1:n)) * U(j, j+1:n)') / D(j,j);
        end
    end
end

function [U_p, D_p] = thornton_update(U, D, PHI, UQ, DQ)
    % Thornton's Zeitpropagation (MWGS) für P = PHI*U*D*U'*PHI' + G*UQ*DQ*UQ'*G'
    % Hier vereinfacht für G=I
    n = size(PHI, 1);
    m = size(UQ, 2);
    W = [PHI * U, UQ]; 
    D_vec = [diag(D); diag(DQ)];
    
    U_p = eye(n); D_p = zeros(n);
    for j = n:-1:1
        for k = 1:n+m
            v(k) = W(j, k);
            a(k) = v(k) * D_vec(k);
        end
        D_p(j,j) = v * a';
        for i = 1:j-1
            for k = 1:n+m
                U_p(i,j) = U_p(i,j) + W(i,k) * a(k);
            end
            U_p(i,j) = U_p(i,j) / D_p(j,j);
            for k = 1:n+m
                W(i,k) = W(i,k) - U_p(i,j) * v(k);
            end
        end
    end
end

function [U_new, D_new, K] = carlson_update(U, D, h, r)
    % Carlson's Rank-1 Update für P_new = P - K*h*P (bzw. Information)
    % Entspricht dem Messupdate-Teil in D'Souza/Zanetti [cite: 1, 4]
    n = length(h);
    f = h * U;
    g = f * D;
    alpha = r + f * g';
    D_new = zeros(n);
    U_new = U;
    
    K = zeros(n, 1);
    for j = 1:n
        alpha_prev = alpha - f(j)*g(j);
        D_new(j,j) = D(j,j) * alpha_prev / alpha;
        gamma = f(j) / alpha_prev;
        for i = 1:j-1
            U_new(i,j) = U(i,j) - gamma * K(i);
        end
        K = K + g(j) * U(:,j);
        alpha = alpha_prev;
    end
    K = K / (r + f * g'); % Korrekter Gain für State-Update
end