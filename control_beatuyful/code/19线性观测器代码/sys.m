%% 使用时需与Observer.m在同一文件夹内

function  dz = sys(t, z)
%% 定义矩阵
  A=[0 1;-1 -0.5];
  B=[0;1];
  C=[1,0];
  D=0;
  u=0;
%% 定义状态空间方程 
  dz_real=A*z(1:2)+B*u;
  y=C*z(1:2)+D*u;
  L=[1.5; -0.75];
  dz_hat=(A-L*C)*z(3:4)+B*u+L*y;
  dz = [dz_real;dz_hat];
 end