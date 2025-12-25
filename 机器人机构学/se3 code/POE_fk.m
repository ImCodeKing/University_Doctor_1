function [g_theta, exp_xi, exp_xi_total] = POE_fk(theta, w, r, v, g0)
% POE 正运动学统一封装
% theta : n×1 符号或数值
% w, r, v : n×1 cell
% g0 : 4×4 初始位姿

n = length(theta);
assert(length(w)==n && length(r)==n && length(v)==n, ...
    'theta, w, r, v 长度不一致');

exp_xi = cell(n,1);
exp_xi_total = eye(4);

for i = 1:n
    % ===== 计算 v_i =====
    if ~isempty(v{i})
        % 手动给 v → 必须是 prismatic
        assert(all(w{i} == 0), ...
            '第 %d 个关节：给了 v，但 w ≠ 0（转动关节不允许）', i);
        v_i = v{i};
    else
        % 自动算 v
        if all(w{i} == 0)
            error('第 %d 个关节：w=0 但未给 v，无法确定移动方向', i);
        end
        v_i = cross(r{i}, w{i});
    end

    % ===== 计算 exp(xi_hat * theta) =====
    exp_xi{i} = simplify(twist_exp(theta(i), w{i}, v_i));
    exp_xi_total = exp_xi_total * exp_xi{i};

    fprintf('exp_xi_%d = \n', i);
    disp(exp_xi{i});
end

exp_xi_total = simplify(exp_xi_total);
% fprintf('exp_xi_total = \n');
% disp(exp_xi_total);

g_theta = simplify(exp_xi_total * g0);
fprintf('g_theta = \n');
disp(g_theta);
end