function cv_kalman_filter_prediction_comparison()
    clc;
    disp('--- Korrigierter Vergleich: KF vs. UDU-KF vs. UDUI (NASA Table I) ---');

    % 1. SYSTEM PARAMETER
    dt = 0.1; 
    Phi = blkdiag([1, dt; 0, 1], [1, dt; 0, 1]);
    inv_Phi = blkdiag([1, -dt; 0, 1], [1, -dt; 0, 1]);
    Q_diag_val = [10.0, 10.0]; 
    G = [0.5*dt^2, 0; dt, 0; 0, 0.5*dt^2; 0, dt];
    Q_sys = G * diag(Q_diag_val) * G';

    % 2. INITIALISIERUNG
    x_init = [10; 2; 0; 0];
    P_init = diag([ 5.0, 1.0, 1.0, 0.1]);

    % Standard KF & UDU-KF
    x_KF = x_init; P_KF = P_init;
    x_UKF = x_init; [U_UKF, D_UKF] = udu_factorization(P_init);

    % UDU-IF (UDUI)
    Y_init = inv(P_init);
    z_UIF = Y_init * x_init;
    [U_UIF, D_UIF] = udu_factorization(Y_init);

    disp(Q_sys);
    
    steps=3;
    for k = 1:steps
        % --- A. Standard KF ---
        x_KF = Phi * x_KF;
        P_KF = Phi * P_KF * Phi' + Q_sys;

        % --- B. UDU-KF (Thornton) ---
        [x_UKF, U_UKF, D_UKF] = thornton(x_UKF, Phi, U_UKF, D_UKF, G, diag(Q_diag_val));

        % --- C. UDU-IF (UDUI) ---
        % 1. Zeitpropagation: Information reduzieren (Prozessrauschen)
        % Laut Paper: G_tilde = inv(Phi)*G
        G_tilde = inv_Phi * G;
        for j = 1:2
            g_j = G_tilde(:, j);
            dq_j = Q_diag_val(j);
            % Rank-1 Update für Y = (I - K*g')*Y
            [U_UIF, D_UIF, z_UIF] = udui_rank1_noise(U_UIF, D_UIF, z_UIF, g_j, dq_j);
        end
        
        % 2. Zustands-Transformation: z = inv(Phi)' * z und Y = inv(Phi)' * Y * inv(Phi)
        z_UIF = inv_Phi' * z_UIF;
        % Transformation der Faktoren U, D via Thornton (mit Q=0)
        [~, U_UIF, D_UIF] = thornton(zeros(4,1), inv_Phi', U_UIF, D_UIF, zeros(4,2), zeros(2,2));
    end

    % REKONSTRUKTION
    P_UKF_res = U_UKF * D_UKF * U_UKF';
    Y_UIF_res = U_UIF * D_UIF * U_UIF';
    P_UIF_res = inv(Y_UIF_res);
    x_UIF_res = Y_UIF_res \ z_UIF;

    fprintf('\nZustandsvektor x (Schritt %d):\n', steps);
    fprintf('Idx |    Standard KF    |      UDU-KF       |      UDU-IF      \n');
    for i=1:4, fprintf('%d   | %15.7f | %17.7f | %17.7f\n', i, x_KF(i), x_UKF(i), x_UIF_res(i)); end
    
    fprintf('\nFrobenius-Fehler P (UDU_KF): %e\n', norm(P_KF - P_UKF_res, 'fro'));
    fprintf('\nFrobenius-Fehler P (UDU_IF): %e\n', norm(P_KF - P_UIF_res, 'fro'));
    disp(P_KF);
    disp(P_UIF_res);
end

function [U, D, z] = udui_rank1_noise(U, D, z, g, dq)
    % Implementiert exakt das Rank-1 Update für UDUI Zeitpropagation
    % Y_plus = (I - K*g')*Y
    n = length(z);
    f = U' * g;
    v = diag(D) .* f;
    alpha = f' * v + 1.0/dq;
    K = (U * v) / alpha;
    
    % Zustandsupdate
    z = z - K * (g' * z);
    
    % Faktorendate (Bierman Downdate Logik)
    % Hier nutzen wir die Symmetrie von (I-Kg')Y(I-Kg')'
    % alpha_prev startet bei der "Messunsicherheit" (hier 1/dq)
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

% Thornton & UDU_factorization bleiben wie zuvor (korrekt implementiert)
function [x, U, D] = thornton(xin, Phi, Uin, Din, Gin, Q)
    n = size(Phi,1); r = size(Gin,2);
    x = Phi * xin;
    W = [Phi * Uin, Gin];
    D_vec = [diag(Din); diag(Q)];
    U = eye(n); D = zeros(n,n);
    for j = n:-1:1
        v = W(j,:); a = v .* D_vec';
        D(j,j) = v * a';
        for i = 1:j-1
            mu = W(i,:) * a';
            U(i,j) = mu / D(j,j);
            W(i,:) = W(i,:) - U(i,j) * v;
        end
    end
end

function [U, D] = udu_factorization(P)
    [rows,cols] = size(P);
    if rows == cols
        m = rows;
        P = (P+P')/2;
        for j = m:-1:1, 
            for i = j:-1:1, 
                sigma = P(i,j); 
                for k = j+1:m, 
                    sigma = sigma - U(i,k)*D(k,k)*U(j,k); 
                end; 
                if i == j 
                    D(j,j) = sigma; 
                    U(j,j) = 1; 
                else 
                U(i,j) = sigma/D(j,j); 
                end; 
            end; 
        end; 
    else
        error('Input matrix to function udu must be square');
    end;
end