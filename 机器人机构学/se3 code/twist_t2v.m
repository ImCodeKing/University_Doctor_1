function v = twist_t2v(t, omega, theta, R)
% t, omega: 3x1
% theta: 标量
% v: 3x1

I = eye(3);
W = hat(omega);

if nargin < 4 || isempty(R)
    R = expm(theta * W);
    disp("R")
    simplify(R)
end

J = (I - R) * W + theta * (omega * omega.');

% 解线性方程 J * v = t
v = J \ t;   % 等价于 inv(J)*t，但数值更稳定
end