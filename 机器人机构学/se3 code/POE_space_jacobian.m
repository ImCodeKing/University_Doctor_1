function Js = POE_space_jacobian(theta, w, r, v)
% 不使用 Adjoint，不显式构造 g
%
% theta : n×1
% w, r, v : n×1 cell
% Js : 6×n

n = length(theta);
assert(length(w)==n && length(r)==n && length(v)==n, ...
    'theta, w, r, v 长度不一致');

Js = sym(zeros(6, n));

% 当前坐标系的旋转与位置（用于算 r_i'）
R = eye(3);
p = sym([0;0;0]);

for i = 1:n

    if ~isempty(v{i})
        %% -------- 平移关节 --------
        % 教材中：轴方向不变，位置无关
        Js(:,i) = [ zeros(3,1); v{i} ];

        % 更新位置（沿当前方向平移）
        p = p + R * v{i} * theta(i);

    else
        %% -------- 转动关节 --------
        omega = w{i};

        % 轴上一点在当前位形下的位置
        r_i_prime = p + R * r{i};

        % 教材公式：xi' = [omega; r' × omega]
        Js(:,i) = [ omega;
                    cross(r_i_prime, omega) ];

        % 更新 R 和 p（用于后续关节）
        R_i = expm(theta(i) * hat(omega));
        p   = p + R * ( eye(3) - R_i ) * cross(omega, r{i});
        R   = R * R_i;
    end
end

Js = simplify(Js);
end
