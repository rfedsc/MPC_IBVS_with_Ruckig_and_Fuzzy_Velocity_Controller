function [J,s_pred_history]=cost_function(tau_seq,s,s_target,Z,Np,Q,R_weight,W_weight,Ts,K,F,current_theta,A,B)
    J1 = 0;
    J2 = 0;
    J3 = 0;
    s_pred = s;
    s_pred_history = zeros(size(s,1),Np+1);
    s_pred_history(:,1) = s;
    error_history = zeros(length(s),Np);
    manipulability_history = zeros(1, Np);
    
    for j= 1:Np
        tau = tau_seq((j-1)*6+1:j*6);
        s_pred = A*s_pred + B * tau;
        s_pred_history(:,j+1) = s_pred;
        e = s_pred - s_target;
        error_history(:,j) = e;
        J1 = J1 + e'*Q*e;
        J2 = J2 + R_weight*(tau'*tau);
    end
    J = J1+J2+J3;
end