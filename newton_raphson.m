clear;

Tinit = [0 0 1 2.7;
    0 1 0 0;
    -1 0 0 0.76;
    0 0 0 1];

R=[1 0 0;0 0 -1; 0 1 0]*rotx(pi/2)*rotz(-pi/6);
P=[1.5;0.4;1.3];
Tdes = [R,P;zeros(1,3),1];


q_init = [0; 0; 0; 0; 0; 0]* pi/180;
[q,lambdalist]=kuka_ik_numerical(Tinit,q_init,Tdes)


[T_N,~,~,~,~] = axisKinematics(Tinit, lambdalist, q, zeros(size(q)), zeros(size(q)))
