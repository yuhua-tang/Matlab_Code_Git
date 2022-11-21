%% 定义仿真区间
tspan =[0 40];
z0 = [0;0];
%% 求解
[t,z]=ode45(@sys,tspan,z0);
hold on; 
plot (t,z(:,1));
grid on;
