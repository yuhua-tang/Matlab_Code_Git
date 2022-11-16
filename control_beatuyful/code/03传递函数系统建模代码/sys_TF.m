clc;clear;close all;
%% 加载Control Package,使用Matlab则注释下1行
%pkg load control
%% 定义G(s)
s = tf('s');
G_s_1 = tf([1 1],[1 5 6])
G_s_2 = (s + 1)/(s^2 + 5*s+ 6)

G_s = (s+1)/(s+3);
%% 定义C(s)
C_s =  tf([1],[1 2]);
%% 闭环传递函数
%% 方法1
Gcl_1 = (G_s*C_s)/(1+C_s*G_s)
minreal(Gcl_1)
%% 方法2
%单位反馈填1，也可以是H(s)
Gcl_2 = feedback(G_s*C_s,1)