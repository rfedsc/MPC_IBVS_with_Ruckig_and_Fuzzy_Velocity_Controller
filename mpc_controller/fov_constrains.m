function [c_fov,ceq] = fov_constrains(tau_seq,s,z_star,Np,Ts,K,F,image_width,image_heigth,A,B)
    %Initialize constraints
    c_fov = [];%Inequality constraints
    ceq = [];%Equality constraints
    s_pred = s;%Initial visual features (normalized coordinates)
    %Camera intrinsic parameters
    fx = K(1,1);
    fy = K(2,2);
    cx = K(1,3);
    cy = K(2,3);
    %Normalized image boundaries
    u_min = -cx/fx;
    u_max = (image_width-cx)/fx;
    v_min = -cy/fy;
    v_max = (image_heigth-cy)/fy;

    for j=1:Np
        %Calculate interaction matrix
        Ls = Calculate_Ls(s_pred,z_star);
        tau = tau_seq((j-1)*6+1:j*6);
        %Update visual features
        s_pred = A*s_pred + B * tau;
        %s_pred = s_pred+Ts*Ls*tau;
        %Extract u, v
        u = s_pred(1:2:end);
        v = s_pred(2:2:end);
        %Add field of view constraints
        c_fov = [c_fov;
            -u+u_min;
            u-u_max;
            -v+v_min;
            v-v_max];
    end

end