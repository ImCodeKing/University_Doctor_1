%% Stewart平台逆运动学求解
% 作者：自动生成
% 功能：根据给定位姿求解6个杆长

clear; clc; close all;

%% 平台参数定义
% 动平台半径和静平台半径
r_p = 9;   % 动平台 |p_i|
r_b = 10;  % 静平台 |b_i|

% 动平台铰点角度 (度)
theta_p = [0, 60, 120, 180, 240, -60];

% 静平台铰点角度 (度)
theta_b = [-15, 75, 105, 195, 225, -45];

%% 计算静平台和动平台铰点在各自坐标系中的位置
% 静平台铰点 B_i (在基坐标系中)
B = zeros(3, 6);
for i = 1:6
    B(:,i) = [r_b * cosd(theta_b(i));
              r_b * sind(theta_b(i));
              0];
end

% 动平台铰点 P_i (在动平台坐标系中)
P_local = zeros(3, 6);
for i = 1:6
    P_local(:,i) = [r_p * cosd(theta_p(i));
                    r_p * sind(theta_p(i));
                    0];
end

%% 定义6个目标位姿
% [x, y, z, roll, pitch, yaw] 角度单位：度
poses = [
    0,  0, 15,  0,  0,  0;
   -5,  0, 15,  0,  0,  0;
    5,  5, 15,  0,  0,  0;
    5,  0, 15, 15,  0,  0;
    5,  2, 15, 10, 10,  0;
    0,  0, 15, 10, 10, 10
];

%% 逆运动学求解
fprintf('========== Stewart平台逆运动学求解结果 ==========\n\n');

leg_lengths = zeros(6, 6);  % 存储每个位姿的6个杆长

for pose_idx = 1:6
    % 提取位姿参数
    x = poses(pose_idx, 1);
    y = poses(pose_idx, 2);
    z = poses(pose_idx, 3);
    roll  = poses(pose_idx, 4);   % 绕X轴
    pitch = poses(pose_idx, 5);   % 绕Y轴
    yaw   = poses(pose_idx, 6);   % 绕Z轴

    % 计算旋转矩阵 (ZYX欧拉角)
    R = rotz(yaw) * roty(pitch) * rotx(roll);

    % 动平台中心位置
    T = [x; y; z];

    % 计算每个杆的长度
    L = zeros(1, 6);
    for i = 1:6
        % 动平台铰点在基坐标系中的位置
        P_global = R * P_local(:,i) + T;

        % 杆向量
        leg_vector = P_global - B(:,i);

        % 杆长
        L(i) = norm(leg_vector);
    end

    leg_lengths(pose_idx, :) = L;

    % 输出结果
    fprintf('位姿 %d: [x=%.1f, y=%.1f, z=%.1f, roll=%.1f°, pitch=%.1f°, yaw=%.1f°]\n', ...
            pose_idx, x, y, z, roll, pitch, yaw);
    fprintf('杆长: L1=%.4f, L2=%.4f, L3=%.4f, L4=%.4f, L5=%.4f, L6=%.4f\n\n', ...
            L(1), L(2), L(3), L(4), L(5), L(6));
end

%% 保存结果供可视化使用
save('stewart_params.mat', 'B', 'P_local', 'poses', 'leg_lengths', 'r_p', 'r_b');

fprintf('参数已保存到 stewart_params.mat\n');

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
