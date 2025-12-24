function exp_xi = twist_exp(theta, omega, v, R)
% 计算扭向量的指数映射 e^(theta * hat(xi))，其中 xi = [omega; v] (6x1)
% 输入：
%   theta: 标量（指数映射的参数）
%   omega: 3x1 向量（扭向量的角速度部分）
%   v: 3x1 向量（扭向量的线速度部分）
%   R: 可选，3x3 旋转矩阵（若已计算 e^(theta*hat(omega))，可直接传入以节省计算）
% 输出：
%   exp_xi: 4x4 齐次变换矩阵（对应 e^(theta*hat(xi))）

I = eye(3);
W = hat(omega);  % 3x3 反对称矩阵（hat算子）

% 计算旋转部分 R = e^(theta*hat(omega))
if nargin < 4 || isempty(R)
    R = expm(theta * W);
end

% 计算J矩阵（对应公式中的 (E - e^θ^ω)ω× + θ ωω^T）
J = (I - R) * W + theta * (omega * omega');  % 3x3 矩阵

% 计算线位移部分：(E - e^θ^ω)(ω×v) + θ ωω^T v = J * v
p = J * v;  % 3x1 线位移向量

% 构造4x4齐次变换矩阵 e^(theta*hat(xi))
exp_xi = [R, p;
          0, 0, 0, 1];
end