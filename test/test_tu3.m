% ------------ 参数配置区 ------------
Np = 10;                            % 预测步长
s = rand(8,1);                      % 当前视觉特征（8维）
target_pixel = [ 600  600 1000 1000; 
                 1000  600 1000  600;];
a = [800    0  800   0;
       0  800  800   0;
       0    0    1   0;];
s_target = Calculate_s(target_pixel,a); %equation 2.9
Z = 5;                              % 特征深度，用于交互矩阵计算
Q = eye(8);                         % 状态误差权重
R_weight = 1;                       % 控制输入权重
W_weight = 0;                       % 可操作性权重（不考虑）
Ts = 0.02;                           %采样时间
K = []; F = [];                     % 非必须参数
current_theta = zeros(6,1);         % 当前关节角度（如有需要）

% B矩阵按Ls计算
Ls0 = Calculate_Ls(s, Z);           % 自定义函数：返回 8×6 交互矩阵
B = Ts * Ls0;                       % 控制输入映射矩阵（8×6）
A = eye(8);                         % 状态矩阵，单位阵

% ------------ 控制输入范围设置 ------------
range = linspace(-1, 1, 30);        % 控制输入范围
[X, Y] = meshgrid(range, range);
J_vals = zeros(size(X));

% ------------ 控制输入分量选择（在这两个方向上变化） ------------
index1 = 1;     % 控制变量 τ_3
index2 = 2;     % 控制变量 τ_6

% ------------ 遍历两维控制输入组合 ------------
for i = 1:length(range)
    for j = 1:length(range)
        tau_seq = zeros(6 * Np, 1);               % 6维输入*Np时域
        tau_seq(index1) = X(i,j);                 % 设置τ_3
        tau_seq(index2) = Y(i,j);                 % 设置τ_6

        % 调用代价函数
        J_vals(i,j) = cost_function(tau_seq, s, s_target, Z, Np, Q, ...
                         R_weight, W_weight, Ts, K, F, current_theta, A, B);
    end
end

% ------------ 绘图（凸性可视化） ------------
figure;
surf(X, Y, J_vals, ...
    'FaceColor', 'interp', ...   % 彩色平滑填充
    'EdgeColor', 'k', ...        % 黑色网格线
    'LineWidth', 0.5);           % 网格线粗细
xlabel(['\tau_' num2str(index1)]);
ylabel(['\tau_' num2str(index2)]);
zlabel('Cost J');
title(['Cost Function Surface: \tau_' num2str(index1) ' vs \tau_' num2str(index2)]);
colormap(jet); colorbar; grid on;
