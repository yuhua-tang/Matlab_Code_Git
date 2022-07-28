clear all;
close all;
obstacleList = [[3,3,1.5];[12,2,3];[3,9,2];[9,11,2]];
start = [1,0];
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

path = rrt_planning(start,goal,param);


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
pause(1);
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