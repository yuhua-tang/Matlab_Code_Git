close all;
P0 = [0 0];
P3 = [6 4];
p0x = P0(1);
p0y = P0(2);
p3x = P3(1);
p3y = P3(2);
p0yaw = -1.57;
p3yaw = -1.57;
dist = sqrt((p3x - p0x)^2 + (p3y - p0y)^2) / 3;
p1x = 3;
p1y = 0;
p2x = 3;
p2y = 4;
    
P1 = [p1x p1y];
P2 = [p2x p2y];

B = [0 0]';
B_derivative = [0 0]';
B_second_derivative = [0,0]';
K = [0];
i = 1;
t = 0;
while t <= 1

    B(:,i) = P0'.*(1-t)^3 + 3*P1'.*t*(1-t)^2 + 3*P2'.*t^2*(1-t) + P3'.*t^3;
    B_derivative(:,i) =  3 * (1 - t)^2*(P1' - P0') + 6 * (1 -t) * t * (P2' - P1') + 3 * t^2 * (P3' - P2');
    B_second_derivative(:,i) = 6*(1-t)*(P2' - 2*P1' + P0') + 6*t*(P3' - 2*P2' + P1');
    %曲率K,直接从轨迹中获取,和用矩阵计算公式算出来一样
    K(i) = (B_derivative(1,i) * B_second_derivative(2,i) - B_second_derivative(1,i) * B_derivative(2,i))/(B_derivative(1,i)^2 + B_derivative(2,i)^2)^(3/2);
    i = i + 1;
    t = t + 0.1;

end

x = B(1,:);
y = B(2,:);

kappa_arr = [];
posi_arr = [];
norm_arr = [];


for num = 2:(length(B)-1)
    x = B(1,num-1:num+1);
    y = B(2,num-1:num+1);
    [kappa,norm_l] = PJcurvature(x,y);
    posi_arr = [posi_arr;[x(2),y(2)]];
    kappa_arr = [kappa_arr;kappa];
    norm_arr = [norm_arr;norm_l];
end
kappa_arr = [kappa_arr(1);kappa_arr;kappa_arr(end)] 

figure(1)
hold on;
plot(B(1,:),B(2,:),'-');
% quiver(posi_arr(:,1),posi_arr(:,2),...
%     kappa_arr.* norm_arr(:,1),kappa_arr.* norm_arr(:,2));
figure(2)
hold on
plot(kappa_arr,'*');
plot(K,'.')

%% obstacle detection range sim or trajectory expansion 
robot_form = [-0.7,0.5;
              0.7,0.5;
              0.7,-0.5;
              -0.7,-0.5]
          
robot_form_2 = [-0.0,0.5;
              0.7,0.5;
              0.7,-0.5;
              -0.0,-0.5]
          
robot_pos = [1;0;pi/6];
robot_global_form = robot2gobal(robot_pos,robot_form)
robot_global_form_2 = robot2gobal(robot_pos,robot_form_2)

robot_form_plot = [robot_form;robot_form(1,:)];
robot_global_form_plot = [robot_global_form;robot_global_form(1,:)];

robot_form_plot_2 = [robot_form_2;robot_form_2(1,:)];
robot_global_form_plot_2 = [robot_global_form_2;robot_global_form_2(1,:)];

figure(3)
hold on
plot(robot_form_plot(:,1),robot_form_plot(:,2),'-r');
plot(robot_global_form_plot(:,1),robot_global_form_plot(:,2),'-b')
axis([-5 5 -5 5])


theta = [];
legnth_path = length(B);
for i = 2:legnth_path
    theta_tmp = atan2(B(2,i) - B(2,i-1),B(1,i)-B(1,i-1));
    theta = [theta;
             theta_tmp];
    
end
theta = [theta(1,:);theta];

figure(4)
hold on;
for i = 1:1:length(B)
    robot_pos = [B(1,i),B(2,i),theta(i)];
    
    robot_global_form = robot2gobal(robot_pos,robot_form);
    robot_global_form_plot = [robot_global_form;robot_global_form(1,:)];
    plot(robot_global_form_plot(:,1),robot_global_form_plot(:,2),'-.b')
    
    robot_global_form_2 = robot2gobal(robot_pos,robot_form_2);
    robot_form_plot_2 = [robot_form_2;robot_form_2(1,:)];
    robot_global_form_plot_2 = [robot_global_form_2;robot_global_form_2(1,:)];
    plot(robot_global_form_plot_2(:,1),robot_global_form_plot_2(:,2),'-.r')
    
    
end



%% aux function
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



function robot_global_form = robot2gobal(robot_position,robot_form)
    robot_global_form = [];
    x = robot_position(1);
    y = robot_position(2);
    yaw = robot_position(3);
    rotation_matrix = [cos(yaw) -sin(yaw);
                       sin(yaw) cos(yaw)];
    [row,col] = size(robot_form);
%     for i = 1:row
%         robot_global_form = [robot_global_form;
%                               (rotation_matrix*[robot_form(i,1);robot_form(i,2)] + [x;y])'];
%     end
    
    for i = 1:row
        global_form_tmp = [robot_form(i,1) * cos(yaw) - robot_form(i,2) * sin(yaw) + x; 
                           robot_form(i,1) * sin(yaw) + robot_form(i,2) * cos(yaw) + y]
        robot_global_form = [robot_global_form;
                              global_form_tmp']
    end
                   
    

end


