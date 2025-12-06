% 设置参数（请根据你的系统实际设置这些参数）
Np = 10;                      % 预测步长
s = rand(6,1);                % 当前视觉特征，6维示例
s_target = zeros(6,1);        % 目标视觉特征
Z = 1;                        % 深度信息
Q = eye(6);                   % 误差权重
R_weight = 1;                 % 控制代价权重
W_weight = 0;                 % 可操作性权重（如果不考虑设为0）
Ts = 0.1;                     % 采样时间
K = [];                       % 如不使用可设为空
F = [];                       % 如不使用可设为空
current_theta = zeros(6,1);   % 当前关节角（可视情况设定）
A = eye(6);                   % 线性预测模型A矩阵（根据实际系统修改）
B = Ts * eye(6);              % 线性预测模型B矩阵（假设单位响应）

% 控制输入空间（两个变量）
range = linspace(-1, 1, 50);  % 控制输入范围
[J_grid_X, J_grid_Y] = meshgrid(range, range);
J_values = zeros(size(J_grid_X));

% 遍历 τ1 和 τ2 构造代价函数表面
for i = 1:length(range)
    for j = 1:length(range)
        tau_test = zeros(6 * Np, 1);        % 控制输入序列初始化为零
        tau_test(1) = J_grid_X(i,j);        % τ1
        tau_test(2) = J_grid_Y(i,j);        % τ2

        % 调用代价函数
        J_values(i,j) = cost_function(tau_test, s, s_target, Z, Np, Q, R_weight, W_weight, Ts, K, F, current_theta, A, B);
    end
end

% 绘制三维图
figure;
surf(J_grid_X, J_grid_Y, J_values);
xlabel('\tau_1');
ylabel('\tau_2');
zlabel('Cost J');
title('Cost Function Surface for (\tau_1, \tau_2)');
grid on;
shading interp;
