clear all;
close all;
obstacleList = [[3,3,1.5];[12,2,3];[3,9,2];[9,11,2]];
start = [2,1];
goal = [15,12];
animation = true;

%set param
param.randArea = [-2,18];
param.maxIter = 200;
param.expandDis = 2.0;
param.goalSampleRate = 10;
param.start = start;
param.goal = goal;
param.obstacleList = obstacleList;
param.animation = animation;


Node.x = 0;
Node.y = 0;
Node.cost = 0;
Node.parent_index = 0;

% path = rrt_planning(start,goal,param);
path = rrt_star_planning(start,goal,param);
% path = informed_rrt_star_planning(start,goal,param);

%% rrt
function path = rrt_planning(start,goal,param)

start_node.x = start(1);
start_node.y = start(2);
start_node.cost = 0;
start_node.parent_index = 0;

goal_node.x = goal(1); 
goal_node.y = goal(2);
goal_node.cost = 0;
goal_node.parent_index = 0;

node_list = [start_node];
path = [];

for i= 1:param.maxIter
    rnd = sample(param);
    n_ind = get_nearest_list_index(node_list,rnd);
    nearestNode = node_list(n_ind);
    
    % steer
    theta = atan2(rnd(2) - nearestNode.y, rnd(1) - nearestNode.x);
    newNode = get_new_node(theta, node_list, n_ind, nearestNode,param);
    
    noCollision = check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y,param);
    if noCollision
        node_list = [node_list;newNode];
        if param.animation
            draw_graph(newNode,node_list,path,param);
        end
        if is_near_goal(newNode,param)
            if check_segment_collision(newNode.x,newNode.y,param.goal(1),param.goal(2),param)
                [lastIndex,c] = size(node_list);
                path = get_final_course(lastIndex,node_list,param);
                pathLen = get_path_len(path);
                sprintf("current path length: %d",pathLen);
                if param.animation
                    draw_graph(newNode,node_list,path,param);
                end
                return;
            end         
        end
    end
end

end

%% rrt*
function path = rrt_star_planning(start,goal,param)
start_node.x = start(1);
start_node.y = start(2);
start_node.cost = 0;
start_node.parent_index = 0;

goal_node.x = goal(1);
goal_node.y = goal(2);
goal_node.cost = 0;
goal_node.parent_index = 0;

node_list = [start_node];
path = [];
lastPathLength = Inf;

for i= 1:param.maxIter
    rnd = sample(param);
    n_ind = get_nearest_list_index(node_list,rnd);
    nearestNode = node_list(n_ind);
    
    % steer
    theta = atan2(rnd(2) - nearestNode.y, rnd(1) - nearestNode.x);
    newNode = get_new_node(theta, node_list, n_ind, nearestNode,param);
    
    noCollision = check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y,param);
    if noCollision
        nearInds = find_near_nodes(newNode,node_list);
        newNode = choose_parent(newNode,nearInds,node_list,param);
        
        node_list = [node_list;newNode];
        
        newNode = rewire(newNode,nearInds,node_list,param);
        
        if param.animation
            draw_graph(newNode,node_list,path,param);
        end
        
        if is_near_goal(newNode,param)
            if check_segment_collision(newNode.x,newNode.y,param.goal(1),param.goal(2),param)
                [lastIndex,c] = size(node_list);
                tmpPath = get_final_course(lastIndex,node_list,param);
                tmpPathLen = get_path_len(tmpPath);
                if lastPathLength > tmpPathLen
                    path = tmpPath;
                    lastPathLength = tmpPathLen;
                    sprintf("current path length: %d",tmpPathLen);
                end
            end         
        end
    end
end


end

%% Informe rrt*
function path = informed_rrt_star_planning(start,goal,param)
start_node.x = start(1);
start_node.y = start(2);
start_node.cost = 0;
start_node.parent_index = 0;

goal_node.x = goal(1);
goal_node.y = goal(2);
goal_node.cost = 0;
goal_node.parent_index = 0;

node_list = [start_node];
path = [];

cBest = Inf;
%compute the sampling space
cMin = sqrt((start_node.x - goal_node.x)^2 + (start_node.y -goal_node.y)^2);
xCenter = [(start_node.x + goal_node.x)/2,(start_node.y + goal_node.y)/2,0];
a1 = [(goal_node.x - start_node.x)/cMin,(goal_node.y - start_node.y)/cMin,0];

e_theta = atan2(a1(2),a1(1));

C = [cos(e_theta), -sin(e_theta), 0;
    sin(e_theta), cos(e_theta), 0;
    0, 0, 1];

for i= 1:param.maxIter
    rnd = informed_sample(cBest,cMin,xCenter,C,param);
    
    n_ind = get_nearest_list_index(node_list,rnd);
    nearestNode = node_list(n_ind);
    
    % steer
    theta = atan2(rnd(2) - nearestNode.y, rnd(1) - nearestNode.x);
    newNode = get_new_node(theta, node_list, n_ind, nearestNode,param);
    
    noCollision = check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y,param);
    if noCollision
        nearInds = find_near_nodes(newNode,node_list);
        newNode = choose_parent(newNode,nearInds,node_list,param);
        
        node_list = [node_list;newNode];
        
        newNode = rewire(newNode,nearInds,node_list,param);
        
        if is_near_goal(newNode,param)
            if check_segment_collision(newNode.x,newNode.y,param.goal(1),param.goal(2),param)
                [lastIndex,c] = size(node_list);
                tmpPath = get_final_course(lastIndex,node_list,param);
                tmpPathLen = get_path_len(tmpPath);
                if tmpPathLen < cBest
                    path = tmpPath;
                    cBest = tmpPathLen;
                    sprintf("current path length: %d",tmpPathLen);
                end
            end         
        end
    end
    
    if param.animation
        draw_graph_informed_RRTStar(xCenter,cBest,cMin,e_theta,rnd,node_list,path,param);
        
    end
end


end



%% function aux
function rnd = sample(param)
if(rand*100 > param.goalSampleRate)
    rnd = [randi([param.randArea(1),param.randArea(2)],1),randi([param.randArea(1),param.randArea(2)],1)];
else
    rnd = param.goal;
end
end

function n_ind = get_nearest_list_index(node_list,rnd)
dlist = [];
[r,c] = size(node_list);
for i = 1:r
    node = node_list(i);
    dlist = [dlist;(node.x -rnd(1))^2 + (node.y - rnd(2))^2];
end
n_ind = find(dlist == min(dlist));
n_ind = n_ind(1);
end

function newNode = get_new_node(theta, node_list, n_ind, nearestNode,param)
newNode = nearestNode;
newNode.x = newNode.x + param.expandDis * cos(theta);
newNode.y = newNode.y + param.expandDis * sin(theta);
newNode.cost = newNode.cost + param.expandDis;
newNode.parent_index = n_ind;
end

function obs_state = check_segment_collision(x1,y1,x2,y2,param)
[r,c] = size(param.obstacleList);
    for i = 1:r
        obs = param.obstacleList(i,:);
        dd = distance_squared_point_to_segment([x1,y1],[x2,y2],[obs(1),obs(2)]);
        if dd < obs(3)^2
            obs_state = false;
            return;
        end
    end
    obs_state = true;
end

%点到直线的距离平方的计算公式
function dist = distance_squared_point_to_segment(v,w,p)
if(v==w)
    dist = (p-v)*(p-v)';
else
    l2 = (w - v)*(w - v)';
    t = max(0,min(1,(p - v)*(w - v)'/l2));
    projection = v + t * (w - v);
    dist = (p - projection)*(p - projection)';
end
end

function draw_graph(rnd,node_list,path,param)
figure(1);
hold on;
plot(rnd.x,rnd.y);
[r,c] = size(node_list);
for i = 1:r
    node = node_list(i);
    if(node.parent_index ~= 0)
        plot([node.x,node_list(node.parent_index).x],[node.y,node_list(node.parent_index).y],"-g");
    end
end
[r,c] = size(param.obstacleList);
for i = 1:r
    obs = param.obstacleList(i,:);
    scatter(obs(1),obs(2),obs(3)*100);
    
end
if ~isempty(path)
    plot(path(:,1),path(:,2));
end

axis([param.randArea(1) param.randArea(2) param.randArea(1) param.randArea(2)]);
grid on;
scatter(param.start(1),param.start(2),100,"r");
scatter(param.goal(1),param.goal(2),100,"k");
pause(0.01);
end

function bool_ans = is_near_goal(node,param)
d = line_cost(node, param.goal);
if d < param.expandDis
    bool_ans = true;
    return;
end
bool_ans = false;

end

function d = line_cost(node1,node2)
d = sqrt((node1.x - node2(1))^2 + (node1.y - node2(2))^2);
end

function path = get_final_course(lastIndex,node_list,param)
path = [[param.goal(1),param.goal(2)]];
while node_list(lastIndex).parent_index ~= 0
    node = node_list(lastIndex);
    path = [path;[node.x,node.y]];
    lastIndex = node.parent_index;
end
path = [path;[param.start(1),param.start(2)]];
end

function pathLen = get_path_len(path)
pathLen = 0;
size_path = size(path);

for i = 2:size_path(1)
    node1_x = path(i,1);
    node1_y = path(i,2);
    node2_x = path(i-1,1);
    node2_y = path(i-1,2);
    pathLen = pathLen + sqrt((node1_x -node2_x)^2 + (node1_y - node2_y)^2); 
end
end

function near_inds = find_near_nodes(newNode,node_list)
[n_node,c] = size(node_list);
r = 50 * sqrt(log(n_node) / n_node);% 检测圆形大小
d_list = [];
for i = 1:n_node
    node = node_list(i);
    d_list = [d_list; (node.x - newNode.x)^2 + (node.y - newNode.y)^2];
end
near_inds = [];
for i = 1:n_node
    if(d_list(i) < r^2)
        near_inds = [near_inds;i];
    end
end
end

function newNode = choose_parent(newNode,nearInds,node_list,param)
if isempty(nearInds)
    newNode = newNode;
    return;
end
dList = [];
[r,c] = size(nearInds);
for i = 1:r
    dx = newNode.x - node_list(i).x;
    dy = newNode.y - node_list(i).y;
    d = hypot(dx,dy);
    theta = atan2(dy,dx);
    if check_collision(node_list(i),theta,d,param)
        dList = [dList;node_list(i).cost + d];
    else
        dList = [dList;Inf];
    end
end
minCost = min(dList);
minInd = find(dList == min(dList));
minInd = minInd(1);
if minCost == Inf
    newNode = newNode;
    return;
end
newNode.cost = minCost;
newNode.parent_index = minInd;
end

function bool_ans = check_collision(nearNode,theta,d,param)
tmpNode = nearNode;
end_x = tmpNode.x + cos(theta)*d;
end_y = tmpNode.y + sin(theta)*d;

bool_ans = check_segment_collision(tmpNode.x,tmpNode.y,end_x,end_y,param);
end

function newNode = rewire(newNode,nearInds,node_list,param)
[n_node,c] = size(node_list);
[r,c] = size(nearInds);
for i = 1:r
    nearNode = node_list(i);
    d = sqrt((nearNode.x - newNode.x)^2 + (nearNode.y + newNode.y)^2);
    s_cost = newNode.cost + d;
    if nearNode.cost > s_cost
        theta = atan2(newNode.y - nearNode.y,newNode.x - nearNode.x);
        if check_collision(nearNode,theta,d,param)
            nearNode.parent = n_node;
            nearNode.cost = s_cost;
        end
    end
end
end

function rnd = informed_sample(cMax,cMin,xCenter,C,param)
if cMax < Inf
    r = [cMax / 2.0,
         sqrt(cMax^2 - cMin^2)/2.0,
         sqrt(cMax^2 - cMin^2)/2.0];
     L = diag(r);
     xBall = sample_unit_ball();
     A = dot(C,L);
     rnd = dot(A,xBall') + xCenter';
     rnd = [rnd(1) rnd(2)];
     
else
    rnd = sample(param);
    
end

end

function uni_circle = sample_unit_ball()
a = rand;
b = rand;
if b < a
    tmp = a;
    a = b;
    b = tmp;
end
sample_point = [b*cos(2 * pi * a / b),(b * sin(2 * pi * a /b))];

uni_circle = [sample_point(1),sample_point(2),0];

end

function draw_graph_informed_RRTStar(xCenter,cBest,cMin,e_theta,rnd,node_list,path,param)
figure(1);
hold on;
plot(rnd(1),rnd(2));
if cBest ~= Inf
    [elat,elon] = plot_ellipse(xCenter, cBest, cMin, e_theta);
    plot(elat,elon);
end

[r,c] = size(node_list);
for i = 1:r
    node = node_list(i);
    if(node.parent_index ~= 0)
        plot([node.x,node_list(node.parent_index).x],[node.y,node_list(node.parent_index).y],"-g");
    end
end

[r,c] = size(param.obstacleList);
for i = 1:r
    obs = param.obstacleList(i,:);
    scatter(obs(1),obs(2),obs(3)*100);
    
end
if ~isempty(path)
    plot(path(:,1),path(:,2));
end

axis([param.randArea(1) param.randArea(2) param.randArea(1) param.randArea(2)]);
grid on;
scatter(param.start(1),param.start(2),100,"r");
scatter(param.goal(1),param.goal(2),100,"k");
pause(0.01);

end

function [elat,elon] = plot_ellipse(xCenter, cBest, cMin, e_theta)
a = sqrt(cBest^ 2 - cMin^2) / 2.0;
b = cBest / 2.0;

cx = xCenter(1);
cy = xCenter(2);

e_theta = -(pi/2 - e_theta);
Rot = [cos(e_theta), -sin(e_theta);
    sin(e_theta), cos(e_theta)];
t = [];
for i = 0:0.1:2*pi
    t = [t;i];
end

[r,c] = size(t);
x = [];
y = [];
for i =1:r
    x=[x;a * cos(t(i))];
    y=[y;b * sin(t(i))]
end

ellipse = Rot*[x';y'];
elat = ellipse(1,:) + cx;
elon = ellipse(2,:) + cy;

end