function T = jr603_fkine(q)
    % DH parameters of the manipulator
    d = [0,0,0,286.5/1000,0,81.5/1000]; % Offset along z-axis
    a = [0,284/1000,-30/1000,0,0,0];         % Offset along x-axis
    alp = [-1.5708, 0, 1.5708, -1.5708, 1.5708, 0]; % Rotation angle along x-axis
    offset = [0,0,0,0,0,0]; % Offset of joint angles
    % Calculate actual angle of each joint
    thd = q + offset; 
    
    % Calculate transformation matrix of each joint
    A1 = trotz(thd(1)) * transl(0, 0, d(1)) * trotx(alp(1)) * transl(a(1), 0, 0);
    A2 = trotz(thd(2)) * transl(0, 0, d(2)) * trotx(alp(2)) * transl(a(2), 0, 0);
    A3 = trotz(thd(3)) * transl(0, 0, d(3)) * trotx(alp(3)) * transl(a(3), 0, 0);
    A4 = trotz(thd(4)) * transl(0, 0, d(4)) * trotx(alp(4)) * transl(a(4), 0, 0);
    A5 = trotz(thd(5)) * transl(0, 0, d(5)) * trotx(alp(5)) * transl(a(5), 0, 0);
    A6 = trotz(thd(6)) * transl(0, 0, d(6)) * trotx(alp(6)) * transl(a(6), 0, 0);

    % Final transformation matrix
    T = A1 * A2 * A3 * A4 * A5 * A6;
end

function T = transl(x, y, z)
    % Generate translation matrix
    T = [1, 0, 0, x;
         0, 1, 0, y;
         0, 0, 1, z;
         0, 0, 0, 1];
end

function R = trotx(theta)
    % Generate transformation matrix for rotation around x-axis
    R = [1, 0, 0, 0;
         0, cos(theta), -sin(theta), 0;
         0, sin(theta), cos(theta), 0;
         0, 0, 0, 1];
end

function R = trotz(theta)
    % Generate transformation matrix for rotation around z-axis
    R = [cos(theta), -sin(theta), 0, 0;
         sin(theta), cos(theta), 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1];
end
%q=[0,0,0,0,0,0]
%T=jr603_fkine_(q)
%disp(T)