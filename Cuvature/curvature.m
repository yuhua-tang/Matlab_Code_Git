close all,clc;
clear all;
P0 = [0 0];
P3 = [6 6];
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
p2y = 6;
    
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

% figure(1)
% hold on;
% plot(B(1,:),B(2,:),'-');
% % quiver(posi_arr(:,1),posi_arr(:,2),...
% %     kappa_arr.* norm_arr(:,1),kappa_arr.* norm_arr(:,2));
% figure(2)
% hold on
% plot(kappa_arr,'*');
% plot(K,'.')

%% obstacle detection range sim or trajectory expansion 
robot_form = [-0.7,0.5;
              0.7,0.5;
              0.7,-0.5;
              -0.7,-0.5];
          
robot_form_2 = [-0.0,0.5;
              0.7,0.5;
              0.7,-0.5;
              -0.0,-0.5];
          
robot_pos = [1;0;pi/6];
robot_global_form = robot2gobal(robot_pos,robot_form)
robot_global_form_2 = robot2gobal(robot_pos,robot_form_2)



robot_form_plot = [robot_form;robot_form(1,:)];
robot_global_form_plot = [robot_global_form;robot_global_form(1,:)];

robot_form_plot_2 = [robot_form_2;robot_form_2(1,:)];
robot_global_form_plot_2 = [robot_global_form_2;robot_global_form_2(1,:)];

% figure(3)
% hold on
% plot(robot_form_plot(:,1),robot_form_plot(:,2),'-r');
% plot(robot_global_form_plot(:,1),robot_global_form_plot(:,2),'-b')
% axis([-5 5 -5 5])


theta = [];
legnth_path = length(B);
for i = 2:legnth_path
    theta_tmp = atan2(B(2,i) - B(2,i-1),B(1,i)-B(1,i-1));
    theta = [theta;
             theta_tmp];
    
end
theta = [theta(1,:);theta];
%{
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
%}

figure(9)
hold on;
robot_pos_init = [B(1,1),B(2,1),theta(1)]
robot_init_global_form = robot2gobal(robot_pos_init,robot_form);

left_start_point = robot_init_global_form(1,1:2);
right_start_point = robot_init_global_form(end,1:2);
for i = 2:1:length(B)
    robot_pos = [B(1,i-1),B(2,i-1),theta(i-1)];
    robot_pos_next = [B(1,i),B(2,i),theta(i)];
    
    robot_global_form = robot2gobal(robot_pos,robot_form);
    robot_global_form_plot = [robot_global_form;robot_global_form(1,:)];
    plot(robot_global_form_plot(:,1),robot_global_form_plot(:,2),'-.b')
    
    robot_global_form_2 = robot2gobal(robot_pos_next,robot_form);
    robot_form_plot_2 = [robot_form;robot_form(1,:)];
    robot_global_form_plot_2 = [robot_global_form_2;robot_global_form_2(1,:)];
    plot(robot_global_form_plot_2(:,1),robot_global_form_plot_2(:,2),'-.r')
    
    new_form = ClockMergeForm(robot_global_form,robot_global_form_2,robot_pos_next);
    new_form_plot = [new_form;new_form(1,:)];
    plot(new_form_plot(:,1),new_form_plot(:,2),'-.k')
    
    if i == 2
        new_form_1 = new_form;
    end
    
    new_form_gravity_center = GetCenterOfGravity(new_form_1);
    
    new_form_1 = ClockMergeForm(new_form_1,robot_global_form_2,new_form_gravity_center);
    new_form_plot_1 = [new_form_1;new_form_1(1,:)];
    
    if i == 2
        auto_merget_form = robot_global_form;
        [row,col] = size(robot_global_form);
        column = zeros(row,1);
        auto_merget_form = [auto_merget_form column];
    end
    robot_global_form_auto_next = robot_global_form_2;
    [row_auto,col_auto] = size(robot_global_form_auto_next);
    column_auto = zeros(row_auto,1);
    robot_global_form_auto_next = [robot_global_form_auto_next column_auto];
    
    
    auto_merget_form = AutoMergeFrom(auto_merget_form,robot_global_form_auto_next,robot_pos,robot_pos_next,left_start_point,right_start_point);
    
 
end


% % test
% new_form_gravity_center_end = GetCenterOfGravity(new_form_1);
% new_form = ClockCheck(new_form_1,new_form_gravity_center_end);
% new_form_plot_1 = [new_form_1;new_form_1(1,:)];
% plot(new_form_plot_1(:,1),new_form_plot_1(:,2),'-.m')
figure(10)
auto_merget_form = [auto_merget_form;auto_merget_form(1,:)];

plot(auto_merget_form(:,1),auto_merget_form(:,2),'-.m')




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
                           robot_form(i,1) * sin(yaw) + robot_form(i,2) * cos(yaw) + y;
                           0]
        robot_global_form = [robot_global_form;
                              global_form_tmp']
    end
                   
    

end

function auto_merge_form = AutoMergeFrom(robot_form1,robot_form2,robot_center1,robot_center2,left_start_point,right_start_point)
    % robot_from1 = [x,y,direction];
    [row1,col1] = size(robot_form1);
    filter_point = [];
    for i = 1:row1
        pt1 = robot_form1(i,:);
        flag_pt1 = PtInPolygon(pt1,robot_form2);
        if flag_pt1 == -1
            filter_point = [filter_point;pt1];
        end
    end
    
    [row2,col2] = size(robot_form2);
    for j = 1:row2
        pt2 = robot_form2(j,:);
        flag_pt2 = PtInPolygon(pt2,robot_form1);
        if flag_pt2 == -1
            filter_point = [filter_point;pt2];
        end
    end
    %已经带有方向的点，不需要再次判断
    [left_point_list,right_point_list] = GetPointDirection(filter_point,robot_center1,robot_center2)
    
    left_point_list_order = GetPointListOrder(left_point_list,left_start_point);
    right_point_list_order = GetPointListOrder(right_point_list,right_start_point);
    
    auto_merge_list = MergeLeftAndRightList(left_point_list_order,right_point_list_order);
    
    auto_merge_form = auto_merge_list;
    

end




function new_form = ClockMergeForm(robot_form1,robot_form2,robot_center)
    [row1,col1] = size(robot_form1);
    filter_point = [];
    for i = 1:row1
        pt1 = robot_form1(i,1:2);
        flag_pt1 = PtInPolygon(pt1,robot_form2);
        if flag_pt1 == -1
            filter_point = [filter_point;pt1];
        end
    end
    
    [row2,col2] = size(robot_form2);
    for j = 1:row2
        pt2 = robot_form2(j,1:2);
        flag_pt2 = PtInPolygon(pt2,robot_form1);
        if flag_pt2 == -1
            filter_point = [filter_point;pt2];
        end
    end

    new_form = ClockCheck(filter_point,robot_center);
end

function flag_pt = PtInPolygon(pt,poly)
    all_left = -1;
    all_right = -1;
    [row,col] = size(poly);
    for i = 1:row-1
        a = poly(i,:);
        b = poly(i+1,:);
        pt;
        flag_orient = orient(poly(i,:),poly(i+1,:),pt);
        if flag_orient > 0
            if all_right > 0
                flag_pt = -1;
                return;
            end
            all_left = 1;
        else
            if all_left > 0
                flag_pt = -1;
                return;
            end
            all_right = 1;
        end
    end
    flag_orient = orient(poly(row,:),poly(1,:),pt);
    if flag_orient > 0
        if all_right > 0
            flag_pt = -1;
            return;
        end
    else
        if all_left > 0
            flag_pt = -1;
            return;
        end
    end
    flag_pt = 1;
end

function orient_ans = orient(a,b,c)
    acx = a(1) - c(1);
    bcx = b(1) - c(1);
    acy = a(2) - c(2);
    bcy = b(2) - c(2);
    orient_ans = acx * bcy - acy * bcx;
    % orient(a,b,c) < 0 Right, orient(a,b,c) > 0 Left
end

function new_form =  ClockCheck(point_list,pt)
    [row,col] = size(point_list);
    new_list = [];
    theta_list = [];
    for i = 1:row
        theta = atan2(point_list(i,2) - pt(2),point_list(i,1) - pt(1));
        theta_list = [theta_list;theta];
        new_list = [new_list;theta,point_list(i,1),point_list(i,2)];
    end
    theta_list = sort(theta_list)
    new_form = [];
    for i = 1:row
        for j = 1:row
            if theta_list(i) == new_list(j,1)
                new_form = [new_form;new_list(j,2:3)];
            end
        end
    end
    new_form
end

function gravity_point = GetCenterOfGravity(poly_list)
    pt1 = poly_list(1,:);
    pt2 = poly_list(2,:);
    sumarea = 0, sumx = 0, sumy = 0;
    
    [row,col] = size(poly_list);
    for i = 3:1:row
        pt3 = poly_list(i,:);
        area = Area(pt1,pt2,pt3);
        sumarea = area + sumarea;
        sumx = (pt1(1) + pt2(1) + pt3(1))*area + sumx;
        sumy = (pt1(2) + pt2(2) + pt3(2))*area + sumy;
        pt2 = pt3;
    end
    cogX = sumx/sumarea/3;
    cogY = sumy/sumarea/3;
    gravity_point = [cogX,cogY];
end

function area = Area(pt0,pt1,pt2)
    area = 0;
%     area = p0.X*p1.Y + p1.X*p2.Y + p2.X*p0.Y - p1.X*p0.Y - p2.X*p1.Y - p0.X*p2.Y;
    area = pt0(1)*pt1(2) + pt1(1)*pt2(2) + pt2(1)*pt0(2) - pt1(1)*pt0(2) - pt2(1)*pt1(2) - pt0(1)*pt2(2);
    area = area/2;
end

function [left_point_list,right_point_list] = GetPointDirection(point_list,pt1,pt2)
    % 0 mean no direction, -1 right, 1 left
    [row,col] = size(point_list);
    left_point_list = [];
    right_point_list = [];
    for i = 1:row
        if point_list(i,3) == 0
            a = pt1;
            b = pt2;
            c = point_list(i,1:2);
            direction = orient(a,b,c);
            if direction > 0
                left_point_list = [left_point_list;point_list(i,1:2), 1,point_list(i,4)];
            end
            if direction < 0
                right_point_list = [right_point_list;point_list(i,1:2), -1,point_list(i,4)];
            end
        end
        if point_list(i,3) == 1
                left_point_list = [left_point_list;point_list(i,:)];                
        end
        
        if point_list(i,3) == -1
                right_point_list = [right_point_list;point_list(i,:)];                
        end
    end
end


function point_list_with_order = GetPointListOrder(point_list,start_point)
    
    [row,col] = size(point_list);
    [max_number,index] =max(point_list(:,4),[],1)
    if max_number ~= 0
        start_point = point_list(index,1:2)
    end
    
    dist_list = [];
    dist_index = [];
    for i = 1:row
        if point_list(i,4) == 0
            tmp_dist = sqrt((start_point(1) - point_list(i,1))^2 + (start_point(2) - point_list(i,2))^2);
            dist_list = [dist_list;tmp_dist];
            dist_index = [dist_index;tmp_dist,i];
        end
    end
    dist_list = sort(dist_list);
    [row2,col2] = size(dist_list);
    tmp_point_list_with_order = [];
    
    max_index_number = max_number;
    for i =1:row2
        for j = 1:row2
            if dist_list(i) == dist_index(j,1)
                max_index_number = max_index_number + 1;
                tmp_point_list_with_order = [tmp_point_list_with_order;point_list(dist_index(j,2),1:3),max_index_number];
            end
        end
    end
    tmp_point_list_with_order
    point_list_with_order = [];
    for i = 1:row
        if point_list(i,4) ~= 0
            point_list_with_order = [point_list_with_order;point_list(i,:)];
        end
    end
    if index == 1
        point_list_with_order = flipud(point_list_with_order);
    end
    
    point_list_with_order = [point_list_with_order;tmp_point_list_with_order]
    
end

function auto_merge_list = MergeLeftAndRightList(left_point_list_order,right_point_list_order)
    auto_merge_list = left_point_list_order;
    right_point_list_order = flipud(right_point_list_order);
    auto_merge_list = [auto_merge_list;right_point_list_order];
end