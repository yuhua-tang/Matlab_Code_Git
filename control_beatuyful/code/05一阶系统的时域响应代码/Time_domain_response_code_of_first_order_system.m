clc;clear;close all;
%% 加载Control Package,使用Matlab则注释下1行
% pkg load control
%% 定义一阶系统
num = [-1 5];
den = [1 5 4];
G_s = tf(num,den);
%% 仿真
subplot(3,1,1)
%% 单位冲激响应
impulse(G_s);
%% 单位阶跃响应
subplot(3,1,2)
step(G_s);
%% 对初始状态的响应
subplot(3,1,3)
% x0 = 10;
% A = -5;
% B = -5;
% C = 1; 
% D = 0;
% sys = ss(A,B,C,D);
% initial(sys,x0);

[A,B,C,D] = tf2ss(num,den)
sys = ss(A,B,C,D);
x0 = eye(size(B))*10
initial(sys,x0);
