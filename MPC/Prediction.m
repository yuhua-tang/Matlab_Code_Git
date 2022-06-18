function u_k= Prediction(x_k,E,H,N,p)
    U_K = zeros(N*p,1);% NP x 1
    U_K = quadprog(H,E*x_k);
    u_k = U_K(1:p,1); % 取第一个结果
end

