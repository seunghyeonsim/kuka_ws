function [f_sols]=kuka_ik(R06,p)
D       = [0.645 0 0 1.220 0 0.240];
A       = [-0.330 1.150 -0.115 0 0 0];    

sols = [];
w6=[0;0;-D(6)];

%% 여기서 부터 풀기 시작
c = p-R06*w6;
c = c';
px=c(1);
py=c(2);
pz=c(3);

d1=D(1);
L2=A(2);
L3=sqrt(D(4)^2+A(3)^2);

%% theta1 구하기(2개)
theta1_1=atan2(py,px);
theta1_2=atan2(-py,-px);
c1_1=cos(theta1_1);
s1_1=sin(theta1_1);
c1_2=cos(theta1_2);
s1_2=sin(theta1_2);

theta1_1=-atan2(py,px);
theta1_2=-atan2(-py,-px);


%% theta3 구하기 (8개)
% theta1_1일때 theta3 
c3_1=((px-0.330*c1_1)^2+(py-0.330*s1_1)^2+(pz-d1)^2-L2^2-L3^2)/(2*L2*L3); % cosine 함수는 + - 값이 같음. 따라서 2개씩 더 나옴.
if c3_1 > 1
    c3_1 = ((px+0.330*c1_1)^2+(p+0.330*s1_1)^2+(pz-d1)^2-L2^2-L3^2)/(2*L2*L3);
end
s3_1=sqrt(1-c3_1^2);
theta3_1_1 = atan2(s3_1,c3_1);
theta3_1_2 = atan2(-s3_1,c3_1);
theta3_1_3 = -theta3_1_1;
theta3_1_4 = -theta3_1_2;

c3_1_1=cos(theta3_1_1);
s3_1_1=sin(theta3_1_1);
c3_1_2=cos(theta3_1_2);
s3_1_2=sin(theta3_1_2);

theta3_1_1 = theta3_1_1 + atan2(0.115,1.220); % 실제 로봇의 각도
theta3_1_2 = theta3_1_2 + atan2(0.115,1.220);
theta3_1_3 = theta3_1_3 + atan2(0.115,1.220);
theta3_1_4 = theta3_1_4 + atan2(0.115,1.220);

% theta1_2일때 theta3 
c3_2=((px-0.330*c1_2)^2+(py-0.330*s1_2)^2+(pz-d1)^2-L2^2-L3^2)/(2*L2*L3);
if c3_2 > 1
    c3_2 = ((px+0.330*c1_2)^2+(py+0.330*s1_2)^2+(pz-d1)^2-L2^2-L3^2)/(2*L2*L3);
end
s3_2=sqrt(1-c3_2^2);
theta3_2_1=atan2(s3_2,c3_2);
theta3_2_2=atan2(-s3_2,c3_2);
theta3_2_3 = -theta3_2_1;
theta3_2_4 = -theta3_2_2;

c3_2_1=cos(theta3_2_1);
s3_2_1=sin(theta3_2_1);
c3_2_2=cos(theta3_2_2);
s3_2_2=sin(theta3_2_2);

theta3_2_1 = theta3_2_1 + atan2(0.115,1.220); % 실제 로봇의 각도
theta3_2_2 = theta3_2_2 + atan2(0.115,1.220);
theta3_2_3 = theta3_2_3 + atan2(0.115,1.220);
theta3_2_4 = theta3_2_4 + atan2(0.115,1.220);

%% theta2 구하기 (4개)
% theta1_1,theta3_1_1 일때 theta2
c2s2=[L2+L3*c3_1_1, L3*s3_1_1; L3*s3_1_1, -(L2+L3*c3_1_1)]\[c1_1*px+s1_1*py-0.330; pz-d1];
c2s2=c2s2';
theta2=atan2(c2s2(2),c2s2(1));
theta(1,:)=[theta1_1,theta2,theta3_1_3];

% theta1_1, theta3_1_2 일때 theta2
c2s2=[L2+L3*c3_1_2, L3*s3_1_2; L3*s3_1_2, -(L2+L3*c3_1_2)]\[c1_1*px+s1_1*py-0.330; pz-d1];
c2s2=c2s2';
theta2=atan2(c2s2(2),c2s2(1));
theta(2,:)=[theta1_1,theta2,theta3_1_4];

% theta1_2,theta3_2_1 and theta3_2_2 일때 theta2
c2s2=[L2+L3*c3_2_1, L3*s3_2_1; L3*s3_2_1, -(L2+L3*c3_2_1)]\[c1_2*px+s1_2*py-0.330; pz-d1];
c2s2=c2s2';
theta2=atan2(c2s2(2),c2s2(1));
theta(3,:)=[theta1_2,theta2,theta3_2_3];

% theta1_2,theta3_2_1 and theta3_2_2 일때 theta2
c2s2=[L2+L3*c3_2_2, L3*s3_2_2; L3*s3_2_2, -(L2+L3*c3_2_2)]\[c1_2*px+s1_2*py-0.330; pz-d1];
c2s2=c2s2';
theta2=atan2(c2s2(2),c2s2(1));
theta(4,:)=[theta1_2,theta2,theta3_2_4];

for i=1:4
    thetalist=theta(i,:);
    if thetalist(1)<=185*pi/180 && thetalist(1)>=-185*pi/180 && (thetalist(2)<=-5*pi/180 && thetalist(2)>=-140*pi/180) && thetalist(3)<=168*pi/180 && thetalist(3)>=-120*pi/180
        sols=[sols;thetalist(1) thetalist(2) thetalist(3)];
    end
end

%% Check
M=[0 0 -1 2.7;  % (1~3)end effector orientation이 theta4 orientation과 같을때
   0 1  0 0;
   1 0  0 0.76;
   0 0  0 1];
Slist=[0 0 -1 0 0 0; 0 1 0 -0.645 0 0.330; 0 1 0 -0.645 0 1.480]';

%% theta4,5,6 구하기
success=0;
for i=1:length(sols(:,1))
    T03=FKinSpace(M,Slist,sols(i,:)');
    R03=T03(1:3,1:3);
    R36=transpose(R03)*R06;

    theta4_1=atan2(R36(2,3),R36(1,3));  % (1~3)end effector orientation이 theta3 orientation과 같을때
    theta5_1=atan2(sqrt(R36(2,3)^2+R36(1,3)^2),R36(3,3));
    theta6_1=atan2(R36(3,2),-R36(3,1));


    if theta4_1<=350*pi/180 && theta4_1>=-350*pi/180 && theta5_1<=122.5*pi/180 && theta5_1>=-122.5*pi/180 && theta6_1<=350*pi/180 && theta6_1>=-350*pi/180
        f_sols(i,:)=[sols(i,:), theta4_1, theta5_1, theta6_1];
        success=1;
        break
    end

    theta4_2=theta4_1+pi;
    theta5_2=-theta5_1;
    theta6_2=theta6_1+pi;
    if theta4_2<=350*pi/180 && theta4_2>=-350*pi/180 && theta5_2<=122.5*pi/180 && theta5_2>=-122.5*pi/180 && theta6_2<=350*pi/180 && theta6_2>=-350*pi/180
        f_sols(i,:)=[sols(i,:), theta4_2, theta5_2, theta6_2];
        success=1;
        break
    end
end