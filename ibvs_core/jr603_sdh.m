figure(1)
title('JR603')
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

JR603=SerialLink([L1 L2 L3 L4 L5 L6],'name','JR603');

%theta_initial = [0*pi/180,-90*pi/180,180*pi/180,0*pi/180,90*pi/180,0*pi/180];
theta_initial = [45*pi/180,-90*pi/180,180*pi/180,0*pi/180,90*pi/180,154*pi/180];
JR603.plot(theta_initial,'scale',0.5);
JR603.teach()