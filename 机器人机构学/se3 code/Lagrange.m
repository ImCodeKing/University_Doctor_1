function [T, M, C, G] = Lagrange(K, P, q, dq, ddq)

n = length(q);
L = K - P;

T = sym(zeros(n,1));

for i = 1:n
    dL_ddq = diff(L, dq(i));

    d_dt_dL_ddq = 0;
    for j = 1:n
        d_dt_dL_ddq = d_dt_dL_ddq ...
            + diff(dL_ddq, q(j)) * dq(j) ...
            + diff(dL_ddq, dq(j)) * ddq(j);
    end

    dL_dq = diff(L, q(i));
    T(i) = simplify(d_dt_dL_ddq - dL_dq);
end

% --- M(q)
M = sym(zeros(n,n));
for i = 1:n
    for j = 1:n
        M(i,j) = diff(T(i), ddq(j));
    end
end
M = simplify(M);

% --- 剩余项
T_no_ddq = simplify(T - M*ddq);

% --- G(q)
G = simplify(subs(T_no_ddq, dq, zeros(n,1)));

% --- C(q,dq)
C_dq = simplify(T_no_ddq - G);
C = sym(zeros(n,n));
for i = 1:n
    for j = 1:n
        C(i,j) = diff(C_dq(i), dq(j));
    end
end
C = simplify(C);

end

