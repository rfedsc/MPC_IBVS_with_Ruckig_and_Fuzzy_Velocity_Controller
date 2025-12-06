function tau = mpc_controller(s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,current_theta,image_width,image_height,tau_now,fig_handle,vis_predict,A,B)
    %参数设置
    v_max = 0.5;%最大线速度
    w_max = 0.5;%最大角速度
    %定义成本函数
    fun = @(tau_seq) cost_function(tau_seq,s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,current_theta,A,B);
    %非线性约束(速度约束和视野约束)
    nonlcon = @(tau_seq)combine_constraints(tau_seq,s,z_star,Np,Ts,K,F,image_width, image_height, v_max, w_max,A,B);
    %定义视野约束
    %nonlcon = @(tau_seq) fov_constrains(tau_seq,s,z_star,Np,Ts,K,F,image_width,image_height);
    %定义优化选项
    options_headless = optimoptions('fmincon', ...
                                   'Display','off', ...
                                   'Algorithm','sqp', ...
                                   'MaxFunctionEvaluations',10000, ...
                                   'MaxIterations',1000,...
                                   'StepTolerance',1e-10,...
                                   'ConstraintTolerance',1e-8);
    options_vis = optimoptions("fmincon", ...
                               "Display","iter",...
                               "Algorithm","sqp", ...
                               "MaxFunctionEvaluations",10000,...
                               'MaxIterations',1000,...
                               'StepTolerance',1e-10,...
                               'ConstraintTolerance',1e-8,...
                               "PlotFcn",{@optimplotx,@optimplotfval});
    if vis_predict
        options = options_vis;
    else
        options = options_headless;
    end

    tau_seq0 = repelem(tau_now,Np,1);
    tau_seq = fmincon(fun,tau_seq0,[],[],[],[],[],[],nonlcon,options);
    %提取第一个控制输入
    tau = tau_seq(1:6);
    [~,s_pred_history] = cost_function(tau_seq,s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,current_theta,A,B);
    if vis_predict
        plot_s_pred_history(s_pred_history,image_width,image_height,fig_handle);
    end

end







