% ------------ 参数配置区 ------------
Np = 10;                            % 预测步长
s = rand(8,1);                      % 当前视觉特征（8维）
target_pixel = [ 600  600 1000 1000; 
                 1000  600 1000  600;];
a = [800    0  800   0;
       0  800  800   0;
       0    0    1   0;];
s_target = Calculate_s(target_pixel,a); %计算目标图像特征
Z = 5;                              % 特征深度，用于交互矩阵计算
Q = eye(8);                         % 状态误差权重
R_weight = eye(6);                       % 控制输入权重
W_weight = 0;                       % 可操作性权重（不考虑）
Ts = 0.02;                           %采样时间
K = []; F = [];                     % 非必须参数
current_theta = zeros(6,1);         % 当前关节角度（如有需要）

% 计算交互矩阵并得到控制输入映射矩阵B
Ls0 = Calculate_Ls(s, Z);           % 自定义函数：返回 8×6 交互矩阵
B = Ts * Ls0;                       % 控制输入映射矩阵（8×6）
A = eye(8);                         % 状态矩阵，单位阵

% 计算代价函数的H矩阵和线性项f
[H, f] = generate_qp_cost(s, s_target, A, B, Q, R_weight, Np);

% 计算Hessian矩阵的特征值
eig_values = eig(H);

% 判断Hessian是否正定
if all(eig_values > 0)
    disp('代价函数是凸的');
else
    disp('代价函数不是凸的');
end

% 创建控制输入的网格
range = linspace(-1, 1, 30); 
[X, Y] = meshgrid(range, range); % 创建控制输入网格
J_vals = zeros(size(X));         % 初始化代价函数值矩阵

% 控制输入分量选择（在这两个方向上变化）
index1 = 3;  % 控制变量 τ_3
index2 = 6;  % 控制变量 τ_6

% 遍历控制输入组合，计算代价函数值
for i = 1:length(range)
    for j = 1:length(range)
        tau_seq = zeros(6 * Np, 1);  % 6维输入 * Np 时域
        tau_seq(index1) = X(i,j);    % 设置τ_3
        tau_seq(index2) = Y(i,j);    % 设置τ_6

        % 代价函数 J(τ) = 1/2 * τ^T H τ + f^T τ
        J_vals(i,j) = 0.5 * tau_seq' * H * tau_seq + f' * tau_seq;
    end
end

% 绘制代价函数的三维表面图
figure;
surf(X, Y, J_vals, 'FaceColor', 'interp', 'EdgeColor', 'k', 'LineWidth', 0.5);
xlabel(['\tau_' num2str(index1)]);
ylabel(['\tau_' num2str(index2)]);
zlabel('代价函数 J');
title(['代价函数表面：\tau_' num2str(index1) ' vs \tau_' num2str(index2)]);
colormap(jet);  % 设置颜色图
colorbar;       % 添加颜色条
grid on;        % 显示网格
