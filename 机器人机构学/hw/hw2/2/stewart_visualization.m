%% Stewart平台3D可视化
% 功能：绘制6个位姿下的机器人形态

clear; clc; close all;

%% 加载参数
load('stewart_params.mat');

%% 创建图形窗口
figure('Name', 'Stewart平台6个位姿可视化', 'Position', [100, 100, 1200, 800]);

%% 绘制6个位姿
for pose_idx = 1:6
    subplot(2, 3, pose_idx);
    hold on;
    grid on;
    axis equal;

    % 提取位姿参数
    x = poses(pose_idx, 1);
    y = poses(pose_idx, 2);
    z = poses(pose_idx, 3);
    roll  = poses(pose_idx, 4);
    pitch = poses(pose_idx, 5);
    yaw   = poses(pose_idx, 6);

    % 计算旋转矩阵
    R = rotz(yaw) * roty(pitch) * rotx(roll);
    T = [x; y; z];

    % 计算动平台铰点全局坐标
    P_global = zeros(3, 6);
    for i = 1:6
        P_global(:,i) = R * P_local(:,i) + T;
    end

    % 绘制静平台（蓝色圆）
    draw_platform(B, 'b', 2);

    % 绘制动平台（红色圆）
    draw_platform(P_global, 'r', 2);

    % 绘制6条腿（绿色）
    for i = 1:6
        plot3([B(1,i), P_global(1,i)], ...
              [B(2,i), P_global(2,i)], ...
              [B(3,i), P_global(3,i)], ...
              'g-', 'LineWidth', 2);
        % 绘制铰点
        plot3(B(1,i), B(2,i), B(3,i), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        plot3(P_global(1,i), P_global(2,i), P_global(3,i), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    end

    % 绘制平台中心
    plot3(0, 0, 0, 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot3(x, y, z, 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    % 设置视角和标签
    view(30, 25);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('位姿%d: [%.0f,%.0f,%.0f,%.0f°,%.0f°,%.0f°]', ...
          pose_idx, x, y, z, roll, pitch, yaw));
    xlim([-15, 15]); ylim([-15, 15]); zlim([0, 25]);
end

sgtitle('Stewart平台逆运动学 - 6个位姿可视化');

%% 保存图片
saveas(gcf, 'stewart_6poses.png');
fprintf('图片已保存为 stewart_6poses.png\n');

%% 辅助函数：绘制平台（连接铰点形成多边形）
function draw_platform(points, color, linewidth)
    % 按顺序连接各点形成封闭多边形
    order = [1, 2, 3, 4, 5, 6, 1];  % 闭合
    plot3(points(1, order), points(2, order), points(3, order), ...
          [color, '-'], 'LineWidth', linewidth);
end

%% 旋转矩阵函数
function R = rotx(angle_deg)
    a = deg2rad(angle_deg);
    R = [1, 0, 0;
         0, cos(a), -sin(a);
         0, sin(a), cos(a)];
end

function R = roty(angle_deg)
    a = deg2rad(angle_deg);
    R = [cos(a), 0, sin(a);
         0, 1, 0;
         -sin(a), 0, cos(a)];
end

function R = rotz(angle_deg)
    a = deg2rad(angle_deg);
    R = [cos(a), -sin(a), 0;
         sin(a), cos(a), 0;
         0, 0, 1];
end
