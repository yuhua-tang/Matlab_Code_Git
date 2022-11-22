function  dz = sys(t, z)
%% 定义参数
  g=10;
  d=1;
%% 定义矩阵
 A=[0 1;g/d 0];
 B=[0;1];
 C = [1, 0];
 D = 0;
 u = 0;
%% 定义状态空间方程 
  dz_real=A*z(1:2)+B*u;
%   y=C*z(1:2)+D*u;
  y = z(1);
  L=[2; 11];
  dz_hat=(A-L*C)*z(3:4)+B*u+L*y;
  dz = [dz_real;dz_hat];
  
  
end

