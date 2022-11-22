%% 使用时需与Tracking.m在同一文件夹内

function  dz = sys(t, z)
%% 定义参数
   g=10;
   d=1;
%% 定义矩阵
   A=[0 1;g/d 0];
   B=[0;1];
   C = [1, 0];
   D = 0; 
%% 定义目标zd
   if t >= 0 & t <10
    z1d = pi/20;
    elseif t >= 20 & t <30
    z1d = -pi/20;
    else
    z1d = 0;
   end
   
%% 使用LQR确定K
sys = ss(A,B,C,D);
[num,den] = ss2tf(A,B,C,D)
G_s=tf(num,den)
%% 定义权重系数，求K
q1=[100 0;0 1];
r1=1;
[K1] = lqr (sys, q1, r1);

%% 定义状态空间方程
   zd = [z1d; 0];
   Ke = [25, 7];
%    Ke = K1;
   F = [-g/d, 0]; 
   e = zd - z; 
   u = F * zd + Ke*e;
   dz = A*z + B*u;
end