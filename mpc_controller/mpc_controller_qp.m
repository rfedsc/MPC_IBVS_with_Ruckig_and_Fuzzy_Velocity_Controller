function tau = mpc_controller_qp(s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,image_width,image_height,tau_now,A,B,v_max,w_max)
    %===================Generate cost function=================
    [H,f] = generate_qp_cost(s,s_target,A,B,Q,R_weight,Np);

    %===================Generate inequality constraints===============
    %1. Velocity constraint: A_vel*tau_seq <= b_vel
    [A_vel,b_vel] = velocity_constraints_qp(Np,v_max,w_max);
    %2. Field of view constraint: G_fov*tau_seq <= h_fov
    [G_fov,h_fov] = get_fov_constrains(s,z_star,Np,K,F,image_width,image_height,A,B);
    disp('size b_vel');
    disp(size(b_vel));
    disp('size h_fov');
    disp(size(h_fov));
    %3. Total inequality constraint: G_all*tau_seq <= h_all
    G_all = [A_vel;G_fov];
    h_all = [b_vel;h_fov];

    %=================Initial guess=====================
    tau_seq0 = repmat(tau_now,Np,1);
    %=================QP solver=====================
    options = optimoptions('quadprog','Display','off');
    [tau_seq,~,exitflag] = quadprog(H,f,G_all,h_all,[],[],[],[],[],options);
    if exitflag ~= 1
        waring('QP failed,using fallback');
        %Degradation processing
        tau = tau_now;
    else
        %Extract the first control step
        tau = tau_seq(1:6);
    end
end