clear all;
clc;
close all;
%生成曲线
P0 = [0 0];
P3 = [6 4];
p0x = P0(1);
p0y = P0(2);
p3x = P3(1);
p3y = P3(2);

dist = sqrt((p3x - p0x)^2 + (p3y - p0y)^2) / 3;
p1x = 3;
p1y = 0;
p2x = 3;
p2y = 4;

p0yaw = atan2(p1y - p0y,p1x - p0x);
p3yaw = atan2(p3y - p2y,p3x - p2x);
    
P1 = [p1x p1y];
P2 = [p2x p2y];

B = [0 0]';
B_derivative = [0 0]';
B_second_derivative = [0,0]';
B_kappa = [0];
i = 1;
t = 0;
while t <= 1

    B(:,i) = P0'.*(1-t)^3 + 3*P1'.*t*(1-t)^2 + 3*P2'.*t^2*(1-t) + P3'.*t^3;
    B_derivative(:,i) =  3 * (1 - t)^2*(P1' - P0') + 6 * (1 -t) * t * (P2' - P1') + 3 * t^2 * (P3' - P2');
    B_second_derivative(:,i) = 6*(1-t)*(P2' - 2*P1' + P0') + 6*t*(P3' - 2*P2' + P1');
    %曲率K,直接从轨迹中获取,和用矩阵计算公式算出来一样
    B_kappa(i) = (B_derivative(1,i) * B_second_derivative(2,i) - B_second_derivative(1,i) * B_derivative(2,i))/(B_derivative(1,i)^2 + B_derivative(2,i)^2)^(3/2);
    i = i + 1;
    t = t + 0.1;

end
%计算曲线heading
B_heading = [];
for i = 1:length(B)-1
    dy = B(2,i+1) - B(2,i);
    dx = B(1,i+1) - B(1,i);
%     dy = B(2,i) - B(2,i + 1);
%     dx = B(1,i) - B(1,i + 1);
    B_heading =[B_heading;atan2(dy,dx)];
    
end
B_heading = [B_heading;p3yaw];

%待投影点
robot_x = 3;
robot_y = 1.8;

%找到匹配点
min_dist_match_point = inf;
min_index_match_point = 1;
for i = 1:length(B)
    dx = B(1,i) - robot_x;
    dy = B(2,i) - robot_y;
    dist_match_point_to_set_point = sqrt(dx^2 + dy^2);
    if dist_match_point_to_set_point < min_dist_match_point
       min_dist_match_point = dist_match_point_to_set_point;
       min_index_match_point = i;
    end
    
end


match_point_x = B(1,min_index_match_point);
match_point_y = B(2,min_index_match_point);
match_point_heading = B_heading(min_index_match_point);
match_point_kappa = B_kappa(min_index_match_point);


%计算匹配点的方向向量与法向量
vector_match_point = [match_point_x;match_point_y];
vector_match_point_direction = [cos(match_point_heading);sin(match_point_heading)];

%声明待投影点的位失
vector_r = [robot_x;robot_y];

%通过匹配点计算投影点
vector_d = vector_r - vector_match_point;
ds = vector_d' * vector_match_point_direction;
vector_proj_point = vector_match_point + ds * vector_match_point_direction
proj_heading = match_point_heading + match_point_kappa * ds
proj_kappa = match_point_kappa

figure(1);
hold on;
scatter(B(1,:),B(2,:),'b');%曲线点
scatter(robot_x,robot_y,'k');%机器人位置
scatter(vector_proj_point(1),vector_proj_point(2),'r');%投影点位置
figure(2)
hold on;
plot(B_kappa,'.')


