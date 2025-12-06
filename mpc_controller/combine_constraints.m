function [c,ceq] = combine_constraints(tau_seq,s,z_star,Np,Ts,K,F,image_width,image_heigth,v_max,w_max,A,B)
    [c_fov,ceq_fov] = fov_constrains(tau_seq,s,z_star,Np,Ts,K,F,image_width,image_heigth,A,B);
    c_vel = velocity_constraints(tau_seq,Np,v_max,w_max);
    c = [c_fov;c_vel];
    ceq = ceq_fov;
end
