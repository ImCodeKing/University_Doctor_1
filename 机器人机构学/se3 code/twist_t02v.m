function v = twist_t02v(t0, omega, theta, R)
% v, omega: 3x1 (可以是符号或数值)
% theta: 标量 (syms 或 double)
% t: 3x1

I = eye(3);
W = hat(omega);

if nargin < 4 || isempty(R)
    R = expm(theta * W);
    disp("R")
    simplify(R)
end

t = R * t0;
disp("t")
simplify(t)

v = twist_t2v(t, omega, theta, R);
end