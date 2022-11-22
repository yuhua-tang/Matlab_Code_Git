% clc;clear;close all;
%% 加载Control Package,使用Matlab则注释下1行
% pkg load control
%% 定义参数
  g=10;
  d=1;
%% 定义矩阵
 A=[0 1;g/d 0];
 B=[0;1];
 C = [1, 0];
 D = 0;

%% 定义初始状态
z0=[pi/20;0];
%% 定义系统控制
% K = [1+g/d 2];
desired_poles = [-2 -2];
K = acker(A,B,desired_poles);

% L = [2 11]';
Desired_poles = [-10 -10];
L = acker(A',C',Desired_poles)';

%%
dt = 0.01;
T = 50;
t = 0:dt:T;
Yr = 0.2*sign(sin(0.5*t));

Acl = A-B*K;
G0 = -C/(A-B*K)*B;

rank(obsv(A,C));

init = [1 1 0 0];
options = odeset('RelTol',1e-2,'AbsTol',1e-4);
tspan = 0:dt:T;
[t,X] = ode45(@(t,x) linear_ode(t,x,A,B,C,K,L),tspan,init,options);
%% PLOTS
plot(t,X(:,1),'b',t,X(:,3),'r',t,0.2*(sin(0.5*t)),'g')
legend('X','Xhat','Yd')
title('X')
xlabel('Time');ylabel('X')

%% FUNCTIONS
function dX = linear_ode(t,XX,A,B,C,K,L)
    Yr = 0.2*(sin(0.5*t));
    G0 = -C/(A-B*K)*B;
    
    X = XX(1:2);
    Xhat=XX(3:4);
    u = -K*Xhat+Yr/G0;
    dX = A*X + B*u;
    Y = C*X;
    Yhat = C*Xhat;
    dXhat = A*Xhat + B*u + L*(Y-Yhat);
    dX=[dX;dXhat];
end


