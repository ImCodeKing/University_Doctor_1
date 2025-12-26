function Js = POE_space_jacobian_test(theta, w, r, v)
% POE 空间雅可比矩阵（Space Jacobian）
% theta : n×1
% w, r, v : n×1 cell
% 输出:
%   Js : 6×n

n = length(theta);
assert(length(w)==n && length(r)==n && length(v)==n, ...
    'theta, w, r, v 长度不一致');

Js = sym(zeros(6, n));
g  = eye(4);      % 累积位姿 g_{i-1}

for i = 1:n
    % ===== 计算 xi_i =====
    if ~isempty(v{i}) & ~all(v{i} == 0)
        % prismatic
        assert(all(w{i} == 0), ...
            '第 %d 个关节：给了 v，但 w ≠ 0（非法）', i);
        omega_i = [0;0;0];
        v_i     = v{i};
    else
        % revolute
        assert(~all(w{i} == 0), ...
            '第 %d 个关节：w = 0 但未给 v（非法）', i);
        omega_i = w{i};
        v_i     = cross(r{i}, w{i});
    end

    xi = [omega_i; v_i];

    % ===== Adjoint(g_{i-1}) =====
    R = g(1:3,1:3);
    p = g(1:3,4);

    Ad_g = [ R, zeros(3);
             hat(p)*R, R ];

    % ===== Js 第 i 列 =====
    Js(:,i) = Ad_g * xi;

    % ===== 更新位姿 =====
    g = g * twist_exp(theta(i), omega_i, v_i);
end

Js = simplify(Js);
end
