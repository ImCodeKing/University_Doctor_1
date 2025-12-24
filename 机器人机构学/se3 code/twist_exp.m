function exp_xi = twist_exp(theta, omega, v, R)
% 计算扭向量指数映射 exp(theta * hat(xi))
% xi = [omega; v]
%
% 适用：
%   - 转动关节 (||omega|| ≠ 0)
%   - 平移关节 (omega = 0)
%
% 输入：
%   theta : 标量
%   omega : 3x1
%   v     : 3x1
%   R     : (可选) 3x3，已计算的 exp(theta*hat(omega))
%
% 输出：
%   exp_xi : 4x4 齐次变换矩阵

I = eye(3);

% 判断是否为纯平移关节
if norm(omega) == 0
    % ===== 纯平移 =====
    R = I;
    p = v * theta;

else
    % ===== 转动关节 =====
    W = hat(omega);

    % 若未提供 R，则计算
    if nargin < 4 || isempty(R)
        R = expm(theta * W);
    end

    % J 矩阵
    J = (I - R) * W + theta * (omega * omega');

    % 位移
    p = J * v;
end

% 构造齐次变换
exp_xi = [R, p;
          0, 0, 0, 1];
end
