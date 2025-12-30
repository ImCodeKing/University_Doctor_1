%% Stewart平台训练数据生成
% 功能：生成大量位姿-杆长数据对，供神经网络训练使用

clear; clc; close all;

%% 平台参数
r_p = 9;   % 动平台半径
r_b = 10;  % 静平台半径
theta_p = [0, 60, 120, 180, 240, -60];
theta_b = [-15, 75, 105, 195, 225, -45];

%% 计算铰点位置
B = zeros(3, 6);
P_local = zeros(3, 6);
for i = 1:6
    B(:,i) = [r_b * cosd(theta_b(i)); r_b * sind(theta_b(i)); 0];
    P_local(:,i) = [r_p * cosd(theta_p(i)); r_p * sind(theta_p(i)); 0];
end

%% 生成训练数据
num_samples = 50000;  % 样本数量
fprintf('正在生成 %d 个训练样本...\n', num_samples);

% 位姿范围定义
x_range = [-8, 8];
y_range = [-8, 8];
z_range = [12, 20];
roll_range = [-20, 20];
pitch_range = [-20, 20];
yaw_range = [-20, 20];

% 随机生成位姿
poses = zeros(num_samples, 6);
poses(:,1) = x_range(1) + (x_range(2)-x_range(1)) * rand(num_samples, 1);
poses(:,2) = y_range(1) + (y_range(2)-y_range(1)) * rand(num_samples, 1);
poses(:,3) = z_range(1) + (z_range(2)-z_range(1)) * rand(num_samples, 1);
poses(:,4) = roll_range(1) + (roll_range(2)-roll_range(1)) * rand(num_samples, 1);
poses(:,5) = pitch_range(1) + (pitch_range(2)-pitch_range(1)) * rand(num_samples, 1);
poses(:,6) = yaw_range(1) + (yaw_range(2)-yaw_range(1)) * rand(num_samples, 1);

% 计算对应的杆长
leg_lengths = zeros(num_samples, 6);

for k = 1:num_samples
    % 提取位姿
    x = poses(k, 1);
    y = poses(k, 2);
    z = poses(k, 3);
    roll = poses(k, 4);
    pitch = poses(k, 5);
    yaw = poses(k, 6);

    % 旋转矩阵
    R = rotz(yaw) * roty(pitch) * rotx(roll);
    T = [x; y; z];

    % 计算6个杆长
    for i = 1:6
        P_global = R * P_local(:,i) + T;
        leg_lengths(k, i) = norm(P_global - B(:,i));
    end

    % 显示进度
    if mod(k, 10000) == 0
        fprintf('已完成 %d / %d\n', k, num_samples);
    end
end

%% 保存数据为CSV文件（供Python使用）
fprintf('正在保存数据...\n');

% 合并数据：[杆长(输入), 位姿(输出)]
% 神经网络：输入=杆长，输出=位姿
data = [leg_lengths, poses];

% 添加表头
header = {'L1','L2','L3','L4','L5','L6','x','y','z','roll','pitch','yaw'};

% 写入CSV
fid = fopen('training_data.csv', 'w');
fprintf(fid, '%s,', header{1:end-1});
fprintf(fid, '%s\n', header{end});
fclose(fid);

dlmwrite('training_data.csv', data, '-append', 'precision', '%.6f');

fprintf('数据已保存到 training_data.csv\n');
fprintf('数据维度: %d 样本 x %d 特征\n', size(data, 1), size(data, 2));

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
