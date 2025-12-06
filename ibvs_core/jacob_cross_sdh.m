function [ J ] = jacob_cross_sdh( q )
d = [0,0,0,286.5/1000,0,81.5/1000];
a = [0,284/1000,-30/1000,0,0,0];
alp = [-1.5708, 0, 1.5708, -1.5708, 1.5708, 0];
offset = [0,0,0,0,0,0];
thd=q+offset;

T0=trotz(0)*transl(0,0,0)*trotx(0)*transl(0,0,0);
T1=trotz(thd(1))*transl(0,0,d(1))*trotx(alp(1))*transl(a(1),0,0);
T2=trotz(thd(2))*transl(0,0,d(2))*trotx(alp(2))*transl(a(2),0,0);
T3=trotz(thd(3))*transl(0,0,d(3))*trotx(alp(3))*transl(a(3),0,0);
T4=trotz(thd(4))*transl(0,0,d(4))*trotx(alp(4))*transl(a(4),0,0);
T5=trotz(thd(5))*transl(0,0,d(5))*trotx(alp(5))*transl(a(5),0,0);
T6=trotz(thd(6))*transl(0,0,d(6))*trotx(alp(6))*transl(a(6),0,0);

T00 = T0;
T01 = T1;
T02 = T1*T2;
T03 = T1*T2*T3;
T04 = T1*T2*T3*T4;
T05 = T1*T2*T3*T4*T5;
T06 = T1*T2*T3*T4*T5*T6;

T06 = T1*T2*T3*T4*T5*T6;
T16 = T2*T3*T4*T5*T6;
T26 = T3*T4*T5*T6;
T36 = T4*T5*T6;
T46 = T5*T6;
T56 = T6;

R00 = t2r(T00);
R01 = t2r(T01);
R02 = t2r(T02);
R03 = t2r(T03);
R04 = t2r(T04);
R05 = t2r(T05);
R06 = t2r(T06);

Z0 = R00(: , 3);
Z1 = R01(: , 3);
Z2 = R02(: , 3);
Z3 = R03(: , 3);
Z4 = R04(: , 3);
Z5 = R05(: , 3);
Z6 = R06(: , 3);

P06 = T06(1:3, 4);
P16 = T16(1:3, 4);
P26 = T26(1:3, 4);
P36 = T36(1:3, 4);
P46 = T46(1:3, 4);
P56 = T56(1:3, 4);
P66 = [0; 0; 0];

J1 = [cross(Z0, R00*P06); Z0];
J2 = [cross(Z1, R01*P16); Z1];
J3 = [cross(Z2, R02*P26); Z2];
J4 = [cross(Z3, R03*P36); Z3];
J5 = [cross(Z4, R04*P46); Z4];
J6 = [cross(Z5, R05*P56); Z5];

J = [J1, J2, J3, J4, J5, J6];

end

function T = trotx(theta)
    T = [1 0 0 0; 0 cos(theta) -sin(theta) 0; 0 sin(theta) cos(theta) 0; 0 0 0 1];
end

function T = trotz(theta)
    T = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
end

function T = transl(x, y, z)
    T = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
end

%q=[10*pi/180,0,0,20*pi/180,0,0];
%T026 = jr603_fkine(q);
%R = T026(1:3,1:3);
%J=jacob_cross_sdh_( q );
%J_end = [R'*J(1:3,:);R'*J(4:6,:)];
%%disp(J)
%disp('J_end is:')
%disp(J_end)
%
%
%
%d = [0,0,0,286.5/1000,0,81.5/1000];
%a = [0,284/1000,-30/1000,0,0,0];
%alp = [-1.5708, 0, 1.5708, -1.5708, 1.5708, 0];
%offset = [0,0,0,0,0,0];
%thd=q+offset;
%%
%L1=Link([0          d(1)       a(1)       alp(1)   ]);L1.offset = offset(1);
%L2=Link([0          d(2)       a(2)       alp(2)   ]);L2.offset = offset(2);
%L3=Link([0          d(3)       a(3)       alp(3)   ]);L3.offset = offset(3);
%L4=Link([0          d(4)       a(4)       alp(4)   ]);L4.offset = offset(4);
%L5=Link([0          d(5)       a(5)       alp(5)   ]);L5.offset = offset(5);
%L6=Link([0          d(6)       a(6)       alp(6)   ]);L6.offset = offset(6);
%
%robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','JR603');
%
%%robot.plot([0,0,0,0,0,0]);
%J1 = robot.jacobe([10*pi/180,0,0,20*pi/180,0,0]);
%disp('J1 is:')
%disp(J1);