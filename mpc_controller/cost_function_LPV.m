function [J,s_pred_history] = cost_function_LPV(tau_seq,s,s_target,Z,Np,Q,R_weight,W_weight,Ts,K,F,current_theta,L_library,rho_samples,sigma)
    J1 = 0; % Tracking error cost
    J2 = 0; % Control input cost
    J3 = 0; % Manipulability index cost
    q = current_theta';
    s_pred = s;
    s_pred_history = zeros(size(s,1),Np+1);
    s_pred_history(:,1) = s; % Initial visual feature vector
    error_history = zeros(length(s),Np);
    manipulability_history = zeros(1,Np); % Manipulability record

    for j = 1:Np
        % Extract current control input from sequence
        tau = tau_seq((j-1)*6+1:j*6);
        % Compute scheduling variable for LPV model
        rho_now = [s_pred;Z];
        % Calculate weighted effective interaction matrix L_eff
        w = compute_weights(rho_now,rho_samples,sigma);
        L_eff = zeros(size(L_library(:,:,1)));
        for i = 1:size(L_library,3)
            L_eff = L_eff + w(i)*L_library(:,:,i);
        end
        % Predict next state using LPV model
        s_pred = s_pred + Ts*L_eff*tau;
        % Record predicted visual feature
        s_pred_history(:,j+1) = s_pred;
        % Calculate tracking error cost
        e = s_pred - s_target;
        error_history(:,j) = e;
        J1 = J1 + e'*Q*e;
        J2 = J2 + R_weight*(tau'*tau);
    end
    % Total cost function value
    J = J1+J2+J3;
end