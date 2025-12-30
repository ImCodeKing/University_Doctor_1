%% Stanford Arm 机器人运动学分析
% 使用Peter Corke Robotics Toolbox
clear; clc; close all;

%% 第一部分: D-H参数
% D-H参数表:
% Joint | Type | d(m)     | theta    | a(m) | alpha
%   1   |  R   | 0.762    | Variable |  0   | -90°
%   2   |  R   | 0.339412 | Variable |  0   | -90°
%   3   |  P   | Variable | -90°     |  0   |   0°
%   4   |  R   | 0.2268   | Variable |  0   | -90°
%   5   |  R   | 0        | Variable |  0   | -90°
%   6   |  R   | 0.4318   | Variable |  0   |   0°

d1 = 0.762; d2 = 0.339412; d4 = 0.2268; d6 = 0.4318;

%% 第二部分: 建立机器人模型
L(1) = Link([0, d1, 0, -pi/2, 0]);
L(2) = Link([0, d2, 0, -pi/2, 0]);
L(3) = Link([-pi/2, 0, 0, 0, 1]);
L(4) = Link([0, d4, 0, -pi/2, 0]);
L(5) = Link([0, 0, 0, -pi/2, 0]);
L(6) = Link([0, d6, 0, 0, 0]);
L(3).qlim = [0.1, 1.5];

stanford = SerialLink(L, 'name', 'Stanford Arm');
disp('Stanford Arm 机器人模型:');
disp(stanford);

%% 第三部分: 定义关节参数
% 位置1: θ1=0°, θ2=-90°, d3=0.635m, θ4=0°, θ5=180°, θ6=180°
q1 = [deg2rad(0), deg2rad(-90), 0.635, deg2rad(0), deg2rad(180), deg2rad(180)];
% 位置4: θ1=20°, θ2=-110°, d3=0.660m, θ4=20°, θ5=170°, θ6=170°
q4 = [deg2rad(20), deg2rad(-110), 0.660, deg2rad(20), deg2rad(170), deg2rad(170)];

%% 第四部分: 正运动学分析 (fkine)
fprintf('\n========== 正运动学分析 ==========\n');
T1 = stanford.fkine(q1);
T4 = stanford.fkine(q4);

fprintf('\n位置1关节参数: θ1=0°, θ2=-90°, d3=0.635m, θ4=0°, θ5=180°, θ6=180°\n');
fprintf('位置1末端位姿矩阵T1:\n'); disp(T1);

fprintf('\n位置4关节参数: θ1=20°, θ2=-110°, d3=0.660m, θ4=20°, θ5=170°, θ6=170°\n');
fprintf('位置4末端位姿矩阵T4:\n'); disp(T4);

%% 第五部分: 可视化机器人构型
figure('Name', '机器人构型', 'Position', [100 100 1000 500]);

subplot(1,2,1);
stanford.plot(q1, 'workspace', [-2 2 -2 2 -1 2.5],'jointdiam', 0.035);
hold on;
trplot(eye(4), 'frame', 'Base', 'color', 'k', 'length', 0.3);
trplot(T1, 'frame', 'EE', 'color', 'r', 'length', 0.2);
title('位置1'); hold off;

subplot(1,2,2);
stanford.plot(q4, 'workspace', [-2 2 -2 2 -1 2.5],'jointdiam', 0.035);
hold on;
trplot(eye(4), 'frame', 'Base', 'color', 'k', 'length', 0.3);
trplot(T4, 'frame', 'EE', 'color', 'r', 'length', 0.2);
title('位置4'); hold off;
sgtitle('Stanford Arm 构型 (黑色:基坐标系, 红色:末端坐标系)');

%% 第六部分: 逆运动学求解 (ikine)
fprintf('\n========== 逆运动学验证 ==========\n');

q1_ik = stanford.ikine(T1, 'q0', q1, 'mask', [1 1 1 1 1 1]);
q4_ik = stanford.ikine(T4, 'q0', q4, 'mask', [1 1 1 1 1 1]);

fprintf('\n位置1逆运动学:\n');
fprintf('原始: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', q1);
fprintf('求解: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', q1_ik);

fprintf('\n位置4逆运动学:\n');
fprintf('原始: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', q4);
fprintf('求解: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', q4_ik);

%% 第七部分: 轨迹规划 (mtraj)
fprintf('\n========== 轨迹规划: 位置1到位置4 ==========\n');
n_steps = 50;
[q_traj, qd, qdd] = mtraj(@tpoly, q1, q4, n_steps);
fprintf('轨迹点数: %d\n', n_steps);

%% 第八部分: 绘制参数空间运动轨迹
figure('Name', '关节空间轨迹', 'Position', [100 100 1000 700]);
t = linspace(0, 1, n_steps);
names = {'\theta_1','\theta_2','d_3','\theta_4','\theta_5','\theta_6'};

for i = 1:6
    subplot(2,3,i);
    plot(t, q_traj(:,i), 'b-', 'LineWidth', 2);
    hold on;
    plot(0, q1(i), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(1, q4(i), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    xlabel('归一化时间');
    if i==3, ylabel('位移(m)'); else, ylabel('角度(rad)'); end
    title(['关节',num2str(i),': ',names{i}]);
    grid on; legend('轨迹','起点','终点','Location','best');
end
sgtitle('位置1到位置4的关节空间轨迹');

%% 第九部分: 末端执行器3D轨迹
xyz = zeros(n_steps, 3);
for i = 1:n_steps
    Ti = stanford.fkine(q_traj(i,:));
    xyz(i,:) = Ti.t';
end

figure('Name', '末端轨迹3D', 'Position', [100 100 800 600]);
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(xyz(1,1), xyz(1,2), xyz(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot3(xyz(end,1), xyz(end,2), xyz(end,3), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
title('末端执行器笛卡尔空间轨迹');
legend('轨迹','位置1','位置4');
grid on; axis equal; view(45,30);

%% 第十部分: 机器人运动动画
fig = figure('Name', '运动动画', 'Position', [100 100 900 750]);
% 添加重播按钮
uicontrol('Style', 'pushbutton', 'String', '重播', ...
    'Position', [400 10 100 30], ...
    'Callback', @(~,~) stanford.plot(q_traj, ...
        'workspace', [-2 2 -2 2 -1 2.5], 'trail', 'b-', 'fps', 20,'jointdiam', 0.035));
% 首次播放
stanford.plot(q_traj, 'workspace', [-2 2 -2 2 -1 2.5], 'trail', 'b-', 'fps', 20,'jointdiam', 0.035);
title('Stanford Arm: 位置1到位置4 (点击下方按钮重播)');

fprintf('\n========== 完成 ==========\n');
