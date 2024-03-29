clc;clear;close all;
path = [1,1;
        5,2;
        7,8];
% path = ginput() * 100.0;

n_order       = 7;              % 多项式的阶数
n_seg         = size(path,1)-1; % 分段数
n_poly_perseg = (n_order+1);    % 每段的系数个数

ts = zeros(n_seg, 1);

%根据距离长度计算每两个点之间的时间分配
dist     = zeros(n_seg, 1);
dist_sum = 0;
T        = 5;
t_sum    = 0;

for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum+dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;
    t_sum = t_sum+ts(i);
end
ts(n_seg) = T - t_sum;

% % 简单地设置每一段的时间分配为1
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

% 分别对x和y方向求取对应的多项式系数,得出对应段的多项式系数从低到高【0,1,2,3,4,5,6,7】7阶8项
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);

% 用于显示轨迹
X_n = [];
Y_n = [];
VX_n = [];
VY_n = [];
Time = [];
k = 1;
tstep = 0.1;
n_order_tmp = n_order;
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis 获得每一段对应的多项式系数
    Pxi = poly_coef_x((n_order_tmp+1)*(i)+1:(n_order_tmp+1)*(i)+n_order_tmp+1)%取出求解对应的一组多项式系数
    Pyi = poly_coef_y((n_order_tmp+1)*(i)+1:(n_order_tmp+1)*(i)+n_order_tmp+1)
    % polyval(p,x)的p是多项式系数系数从最高阶到最低阶
    for t = 0:tstep:ts(i+1)
        %对每一段多项式系数生成对应离散轨迹曲线
        PoyPxi = flip(Pxi);
        PoyPyi = flip(Pyi);
        
        X_n(k)  = polyval(PoyPxi, t);
        Y_n(k)  = polyval(PoyPyi, t);
        
        PoyVxi = polyder(PoyPxi);
        PoyVyi = polyder(PoyPyi);
        
        VX_n(k) = polyval(PoyVxi,t);
        VY_n(k) = polyval(PoyVyi,t);
        if(i > 0)
            Time(k) = t + ts(i);
        else
            Time(k) = t;
        end
        k = k + 1;
    end
    
end
figure(5);
plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
% plot(X_n,'Color', [0 1.0 0], 'LineWidth', 2);
hold on
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));
title('Position Figure');

figure(2);
hold on
plot(Time, VX_n,'-r');
plot(Time, VY_n,'-b');
title('Velocity Figure');
xlabel('Time');
ylabel('Velocity');
legend('vx','vy');

linear_velocity = sqrt(VX_n.^2 + VY_n.^2);

kappa_arr = [];
posi_arr = [];
norm_arr = [];
for num = 2:(length(X_n)-1)
    x = X_n(num-1:num+1);
    y = Y_n(num-1:num+1);
    [kappa,norm_l] = PJcurvature(x,y);
    posi_arr = [posi_arr;[x,y]];
    kappa_arr = [kappa_arr;kappa];
    norm_arr = [norm_arr;norm_l];
end
kappa_arr = [kappa_arr(1);kappa_arr;kappa_arr(end)];
radius = 1./kappa_arr;
angular_velocity = linear_velocity' ./ radius;

% figure(3)
% hold on;
% % plot(X_n,Y_n,'-');
% plot(kappa_arr,'*');



%% 
% TODO:计算出长度，将Vx, Vy转换成V，Omega
path_length = [];
path_theta = [];
path_step_dt = [];
for i = 2:(length(linear_velocity))
    dist =sqrt((X_n(i) - X_n(i-1))^2 + (Y_n(i) - Y_n(i-1))^2);
    theta = atan2(Y_n(i) - Y_n(i-1),X_n(i) - X_n(i-1));
    dt = dist/linear_velocity(i);
    path_length = [path_length;dist];
    path_theta = [path_theta;theta];
    path_step_dt = [path_step_dt;dt];
end
path_length = [path_length;path_length(end)];
path_theta = [path_theta;path_theta(end)];
path_step_dt = [path_step_dt;0];
linear_v = [];
angular_w = [];
omega_w = [];
for i = 1:length(linear_velocity)
    [linear_v_tmp,angular_w_tmp] = tranVXVY2VW(VX_n(i),VY_n(i),path_theta(i),path_length(i))
    omega_w_tmp = getOmega(linear_velocity(i),path_theta(i),path_length(i),VX_n(i));
    linear_v = [linear_v;linear_v_tmp];
    angular_w = [angular_w;angular_w_tmp];
    omega_w = [omega_w;omega_w_tmp];
   
end


% TODO: sim 利用获得线速度，角速度根据差分模型获得轨迹图
robot_pose = [path(1,:), 0];
delta_t = tstep;
robot_pose_record = [];
for i = 1:(length(linear_velocity))
   robot_pose = kinematic(robot_pose,linear_velocity(i),-angular_velocity(i),path_step_dt(i));
   robot_pose_record = [robot_pose_record;
                        robot_pose'];
end

robot_pose_1 = [path(1,:), 0];
delta_t = tstep;
robot_pose_record_1 = [];
for i = 1:(length(linear_velocity))
   robot_pose_1 = kinematic(robot_pose_1,linear_velocity(i),-angular_velocity(i),delta_t);
   robot_pose_record_1 = [robot_pose_record_1;
                        robot_pose_1'];
end

%画线速度和角速度
figure(4)
hold on;
plot(linear_velocity);
plot(angular_velocity);
plot(linear_v);
plot(angular_w);
plot(omega_w);
legend('linear-velocity','angular-velocity','linear-v','angular-w','omega-w')
%画位置
robot_pose_2 = [path(1,:), 0];
robot_pose_record_2 = [];
for i = 1:(length(linear_velocity))
   robot_pose_2 = kinematic(robot_pose_2,linear_v(i),-angular_w(i),path_step_dt(i));
   robot_pose_record_2 = [robot_pose_record_2;
                          robot_pose_2'];
end

figure(5)
plot(robot_pose_record(:,1),robot_pose_record(:,2))
plot(robot_pose_record_1(:,1),robot_pose_record_1(:,2))
plot(robot_pose_record_2(:,1),robot_pose_record_2(:,2))
legend('path-ref','point-position','sim-pos-1','sim-pos-2','sin-pos-3')

%%
% Minisnap求解器
function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    % 起点约束
    start_cond = [waypoints(1), 0, 0, 0];
    % 终点约束
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: 计算Q矩阵
    Q = getQ(n_seg, n_order, ts);
    
    %#####################################################
    % STEP 2: 计算对应的约束矩阵A_beq
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    
    f = zeros(size(Q,1),1);
    % 求解多项式系数
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end

function [kappa,norm_k] = PJcurvature(x,y)
    x = reshape(x,3,1);
    y = reshape(y,3,1);
    t_a = norm([x(2)-x(1),y(2)-y(1)]);
    t_b = norm([x(3)-x(2),y(3)-y(2)]);
    
    M =[[1, -t_a, t_a^2];
        [1, 0,    0    ];
        [1,  t_b, t_b^2]];

    a1 = M\x;
    b2 = M\y;
    a = inv(M)*(x);
    b = inv(M)*(y);
    

    kappa  = 2.*(a(3)*b(2)-b(3)*a(2)) / (a(2)^2.+b(2)^2.)^(1.5);
    norm_k =  [b(2),-a(2)]/sqrt(a(2)^2.+b(2)^2.);
end

function [next_pose] = kinematic(current_pose, velelocity, omega, deltaT)
 next_pose = [current_pose(1) + velelocity * deltaT * cos(current_pose(3) + omega * deltaT);
              current_pose(2) + velelocity * deltaT * sin(current_pose(3) + omega* deltaT);
              current_pose(3) + omega * deltaT];
end

function [v,omega] = tranVXVY2VW(VX,VY,theta,L)
    if (L - 0) < 0.00001
        L = 0.000001
    end
%     L = 0.01; %L 要足够短
    L_matrix = [1 0; 
                0 1/L];
    theta = -theta;
    R_matrix = [cos(theta) -sin(theta);
                sin(theta) cos(theta)];
    trans_ans = L_matrix * R_matrix * [VX;VY];
    v = trans_ans(1);
    omega = trans_ans(2);
end

function omega = getOmega(V,theta,L,Vx)
    omega = (V * cos(theta) - Vx)/(L*sin(theta));
end