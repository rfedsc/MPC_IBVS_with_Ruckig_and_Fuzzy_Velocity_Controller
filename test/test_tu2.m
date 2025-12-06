% ------------ 参数配置区 ------------
Np = 10;                      % 预测步长
s = rand(6,1);                % 当前视觉特征（6维）
s_target = zeros(6,1);        % 目标视觉特征
Z = 1;                        % 深度
Q = eye(6);                   % 状态误差权重
R_weight = 1;                 % 控制输入权重
W_weight = 0;                 % 可操作性权重（此处不考虑）
Ts = 0.1;                     % 采样时间
K = [];                       % 非必须参数
F = [];                       % 非必须参数
current_theta = zeros(6,1);   % 当前关节角
A = eye(6);                   % 状态转移矩阵
B = Ts * eye(6);              % 控制矩阵

% ------------ 控制输入变量范围设置 ------------
range = linspace(-1, 1, 30);  % 控制输入取值范围
[X, Y] = meshgrid(range, range);
J_vals = zeros(size(X));

% ------------ 设置要分析的两个控制输入位置 ------------
index1 = 3;  % 例如 tau_3
index2 = 6;  % 例如 tau_6

% ------------ 遍历两个控制输入维度 ------------
for i = 1:length(range)
    for j = 1:length(range)
        tau_seq = zeros(6 * Np, 1);
        tau_seq(index1) = X(i,j);   % 设置 tau_3
        tau_seq(index2) = Y(i,j);   % 设置 tau_6

        % 调用代价函数
        J_vals(i,j) = cost_function(tau_seq, s, s_target, Z, Np, Q, R_weight, W_weight, Ts, K, F, current_theta, A, B);
    end
end

figure;
surf(X, Y, J_vals, 'FaceColor', 'interp', ...     % 彩色填充
     'EdgeColor', 'k', ...                         % 黑色网格线
     'LineWidth', 0.5);                            % 网格线宽度
xlabel(['\tau_' num2str(index1)]);
ylabel(['\tau_' num2str(index2)]);
zlabel('Cost J');
title(['Cost Function Surface: \tau_' num2str(index1) ' vs \tau_' num2str(index2)]);
colormap(jet);      % 使用 jet 色图增强视觉效果
colorbar;           % 显示颜色条
grid on;

