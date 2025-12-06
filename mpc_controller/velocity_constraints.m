function c_vel = velocity_constraints(tau_seq,Np,v_max,w_max)
%tau_seq is a vector of size Np*6, containing velocity control inputs for Np steps
    c_vel = [];
    for j=1:Np
        %Velocity at each step is tau
        tau = tau_seq((j-1)*6+1:j*6);
        v_xyz = tau(1:3);%Linear velocity
        w_xyz = tau(4:6);%Angular velocity

        %Add velocity inequality constraints
        c_vel = [c_vel;
            v_xyz - v_max;
            -v_xyz - v_max;
            w_xyz - w_max;
            -w_xyz - w_max];
    end
end