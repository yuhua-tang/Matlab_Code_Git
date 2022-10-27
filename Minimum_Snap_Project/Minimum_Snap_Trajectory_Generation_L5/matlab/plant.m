 
%%
 
 
function [xyphi]=plant()
 
clear all;
close all;
clc;
 
%%
 
l=2.9;             %轴距
ts=0.001;
t=0:ts:50;      %定义时间序列
v(1:length(t))=5;  %定义速度序列，匀速5m/s
delta_deg=5*sin(0.5*t)+1*sin(0.2*t);  % 前轮转角  单位为°
delta_rad=(delta_deg/180)*pi;         % 前轮转角  单位为rad
%%
 
delta=delta_rad;
%初始值
x(1)=0;
y(1)=0;
phi(1)=0;
x0=x(1);
y0=y(1);
phi0=phi(1);
for j=1:1:length(v)
   times(j+1)=j*ts; 
   x(j+1)=x0+v(j)*cos(phi0)*ts; 
   y(j+1)=y0+v(j)*sin(phi0)*ts;
   phi(j+1)=phi0+v(j)*tan(delta(j))/l*ts;
   phi0=phi(j+1);
   x0=x(j+1);
   y0=y(j+1);
end
xyphi=[x;y;phi];%后轴中心点位置和横摆角
figure(1);
 
plot(x,y);
axis equal;
 
end