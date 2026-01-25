function [U, D] = udu_factorization(P)
    % UDU factorization
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
