function Js = POE_space_jacobian(theta, w, r, v, sign)
% 不使用 Adjoint，不显式构造 g
%
% theta : n×1
% w, r, v : n×1 cell

n = length(theta);
assert(length(w)==n && length(r)==n && length(v)==n, ...
    'theta, w, r, v 长度不一致');

signed_exp_W = cell(n, 1);
total_exp_W = cell(n, 1);
r_pri = cell(n, 1);
w_pri = cell(n, 1);
v_pri = cell(n, 1);
xi_pri = cell(n, 1);

Js = sym(zeros(6, n));

for i = 1:n
    signed_exp_W{i} = expm(sign{i} * theta(i) * hat(w{i}));
    simplify(signed_exp_W{i});
end

total_exp_W{1} = signed_exp_W{1};
w_pri{1} = w{1};
r_pri{1} = r{1};
v_pri{1} = v{1};

for i = 2:n
    total_exp_W{i} = total_exp_W{i-1} * signed_exp_W{i};
    simplify(total_exp_W{i});
end
for i = 2:n
    r_pri{i} = r{1};
    for j = 2:i
        r_pri{i} = r_pri{i} + total_exp_W{j-1} * r{j};
    end
    simplify(r_pri{i});

    if ~isempty(v{i})
        assert(all(w{i} == 0), ...
            '第 %d 个关节：给了 v，但 w ≠ 0（转动关节不允许）', i);

        w_pri{i} = [0;0;0];
        v_pri{i} = total_exp_W{i - 1} * v{i};
    else
        if all(w{i} == 0)
            error('第 %d 个关节：w=0 但未给 v，无法确定移动方向', i);
        end

        w_pri{i} = total_exp_W{i - 1} * w{i};
        v_pri{i} = cross(r_pri{i}, w_pri{i});
    xi_pri{i} = [w_pri{i}; v_pri{i}];
    simplify(xi_pri{i});
    end
end

for i = 1:n
    Js(:, i) = [w_pri{i}; v_pri{i}];
    w_pri{i};
    r_pri{i};
    v_pri{i};
    signed_exp_W{i};
end

