%% ʹ��ʱ����Observer.m��ͬһ�ļ�����

function  dz = sys(t, z)
%% �������
  A=[0 1;-1 -0.5];
  B=[0;1];
  C=[1,0];
  D=0;
  u=0;
%% ����״̬�ռ䷽�� 
  dz_real=A*z(1:2)+B*u;
  y=C*z(1:2)+D*u;
  L=[1.5; -0.75];
  dz_hat=(A-L*C)*z(3:4)+B*u+L*y;
  dz = [dz_real;dz_hat];
 end