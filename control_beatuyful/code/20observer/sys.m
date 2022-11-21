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
   
%% 定义Observer矩阵
  Obs_A=[0 1;-1 -0.5];
  Obs_B=[0;1];
  Obs_C=[1,0];
  Obs_D=0;
  Obs_u=0;
%% 定义Observer状态空间方程 
  dz_real=Obs_A*z+Obs_B*Obs_u;
  y=Obs_C*z+Obs_D*Obs_u;
  L=[1.5; -0.75];
  dz_hat=(Obs_A-L*Obs_C)*z+Obs_B*Obs_u+L*y;
  obs_dz = dz_hat;
%   obs_dz = [dz_real(1);dz_hat(2)];

%% 定义状态空间方程
   zd = [z1d; 0];
   Ke = [25, 7];
   F = [-g/d, 0]; 
   e = zd - obs_dz;

   u = F * zd + Ke*e;
   dz = A*obs_dz + B*u;
end

