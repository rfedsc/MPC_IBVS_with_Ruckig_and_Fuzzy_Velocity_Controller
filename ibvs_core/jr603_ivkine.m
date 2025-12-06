function joint_velocity=jr603_ivkine(V_b_e,joint_theta_current,joint_velocity_max)
d = [0,0,0,286.5/1000,0,81.5/1000];
a = [0,284/1000,-30/1000,0,0,0];
alp = [-1.5708, 0, 1.5708, -1.5708, 1.5708, 0];
offset = [0,0,0,0,0,0];

L1=Link([0          d(1)       a(1)       alp(1)   ]);L1.offset = offset(1);
L2=Link([0          d(2)       a(2)       alp(2)   ]);L2.offset = offset(2);
L3=Link([0          d(3)       a(3)       alp(3)   ]);L3.offset = offset(3);
L4=Link([0          d(4)       a(4)       alp(4)   ]);L4.offset = offset(4);
L5=Link([0          d(5)       a(5)       alp(5)   ]);L5.offset = offset(5);
L6=Link([0          d(6)       a(6)       alp(6)   ]);L6.offset = offset(6);

robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','JR603');
J = robot.jacobe(joint_theta_current);

cond_number = cond(J);
if cond_number > 100
    warning('机械臂处于奇异位形，数值不稳定！');
    [U, S, V] = svd(J);
    threshold = 1e-6;
    S_inv = diag(1 ./ (diag(S) + threshold));
    J_pinv = V * S_inv * U';
    joint_velocity = J_pinv * V_b_e;
else
    joint_velocity = pinv(J) * V_b_e;
end

joint_velocity = pinv(J)*V_b_e;
joint_velocity = joint_velocity'

end

function J_numeric = jacob_diff(q)
    epsilon = 1e-6;
    J_numeric = zeros(6, 6);
    for i = 1:6
        dq = zeros(1, 6);
        dq(i) = epsilon;
        T_plus = jr603_fkine(q + dq);
        T_minus = jr603_fkine(q - dq);
        pos_plus = T_plus(1:3, 4);
        pos_minus = T_minus(1:3, 4);
        
        rot_plus = rotm2eul(T_plus(1:3,1:3),'ZYX')
        rot_minus = rotm2eul(T_minus(1:3,1:3),'ZYX');
        
        diff_pos = (pos_plus - pos_minus) / (2 * epsilon);
        diff_rot = (rot_plus - rot_minus) / (2 * epsilon);
        
        J_numeric(:, i) = [diff_pos; diff_rot(:)];
    end
end

%q = [30*pi/180,0,40*pi/180,50*pi/180,0,0];
%J1 = jacob_diff(q);
%disp('Numeric Jacobian:');
%disp(J1);