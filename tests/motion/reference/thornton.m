function [x, U, D] = thornton(xin, Phi, Uin, Din, Gin, Q)
    % Thornton covariance time-update
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
