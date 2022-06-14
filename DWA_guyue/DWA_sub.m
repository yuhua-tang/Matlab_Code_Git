clear;close all;
rows = 20; cols = 20;                           % 地图行列尺寸
startSub = [15,3];                              % 起点行列位置
goalSub = [18,19];                               % 终点行列位置
dy_obsSub = [4,4; 13,5; 9,14; 2,18; 12,16];     % 动态障碍物行列
step = 0;                                       % 动态障碍物更新运动的频率

sta_obsSub = [15,10;10,10;10,10;11,10;12,10;13,10;9,10;14,10;8,10;7,10];       %静态障碍物

Path_plan_list = [];
Path_plan_list = AstarSearch(startSub,goalSub,sta_obsSub,rows,cols);
Path_list = [Path_plan_list(1:end,1),Path_plan_list(1:end,2)];

% 设置地图属性
field = ones(rows, cols);


Path_listR = Path_list(:,1);Path_listC = Path_list(:,2);
Path_listIndex = sub2ind([rows,cols],Path_listR,Path_listC);
field(Path_listIndex) = 6;

dy_obsR = dy_obsSub(:,1);dy_obsC = dy_obsSub(:,2);
dy_obsIndex = sub2ind([rows,cols],dy_obsR,dy_obsC);
field(dy_obsIndex) = 3;

sta_obsR = sta_obsSub(:,1);sta_obsC = sta_obsSub(:,2);
sta_obsIndex = sub2ind([rows,cols],sta_obsR,sta_obsC);
field(sta_obsIndex) = 2;

field(startSub(1),startSub(2)) = 4;
field(goalSub(1),goalSub(2)) = 5;

% 颜色表征矩阵
cmap = [1 1 1; ...       % 1-白色-空地
    0 0 0; ...           % 2-黑色-静态障碍
    1 0 0; ...           % 3-红色-动态障碍
    1 1 0;...            % 4-黄色-起始点
    1 0 1;...            % 5-品红-目标点
    0 1 0; ...           % 6-绿色-到目标点的规划路径
    0 1 1];              % 7-青色-动态规划的路径
colormap(cmap);
image(1.5,1.5,field);

% 设置栅格属性
grid on;hold on;
set(gca,'gridline','-','gridcolor','k','linewidth',0.5,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
set(gca, 'XAxisLocation','top')
axis image;

% 机器人现在状态
% 对于DWA算法，结算结果建立在XY坐标系上，先将数据转成XY格式
% 坐标系只影响位置，对于机器人其他运动学不影响
robotXY = sub2coord(startSub);
goalCoord = sub2coord(goalSub);
dy_obsCoord = sub2coord(dy_obsSub);
sta_obsCoord = sub2coord(sta_obsSub);
path_listCoord = sub2coord(Path_list);


robotT = 0;                      % 机器人当前方向角度(0->2pi)
robotV = 0;                         % 机器人当前速度
robotW = 0;                         % 机器人当前角速度
obstacleR=0.6;                      % 冲突判定用的障碍物半径
dt = 0.1;                           % 时间[s]

% 机器人运动学模型
maxVel = 1.5;                       % 机器人最大速度m/s
maxRot = 20.0/180*pi;               % 机器人最大角速度rad/s
maxVelAcc = 0.3;                    % 机器人最大加速度m/ss
maxRotAcc = 50.0/180*pi;            % 机器人最大角加速度rad/ss

% 评价系数
alpha = 0.05;                       % 方位角评价系数α
beta = 0.2;                         % 空隙评价系数β
gama = 0.8;                         % 速度评价系数γ
path_cofe = 0.1;                    % 轨迹评价系数
periodTime = 3;                   % 预测处理时间,也就是绿色搜寻轨迹的长度

path = [];                          % 记录移动路径
vel_cmd = [];                       % 记录速度控制指令信息

%% 开始DWA算法求解
% while step<100
while true
    
    % 是否到达目的地，到达目的地则跳出循环
    if norm(robotXY-goalCoord) < 0.5
        break;
    end
    
    %% 1、求速度区间==============================================
    vel_rangeMin = max(0, robotV-maxVelAcc*dt);
    vel_rangeMax = min(maxVel, robotV+maxVelAcc*dt);
    rot_rangeMin = max(-maxRot, robotW-maxRotAcc*dt);
    rot_rangeMax = min(maxRot, robotW+maxRotAcc*dt);
    
    % 存放当前机器人状态的各个线速度角速度下的评价函数的值
    % evalDB格式为n*6，其中6列为下一状态速度、角速度、方向角函数、距离函数、速度函数、评价函数值
    evalDB = [];
    
    %% 2、计算不同线速度角速度下的评价函数，目的取出最合适的线速度和角速度******************
    for temp_v = vel_rangeMin : 0.01 : vel_rangeMax
        for temp_w = rot_rangeMin : pi/180 : rot_rangeMax
            %% 2.1 最关键内容，不同线速度角速度都是建立在机器人最初始状态下的
            rob_perState = [robotXY(1),robotXY(2),robotT,robotV,robotW]';
            
            %% 2.2 求出在sim_period时间后机器人的状态
            for time = dt:dt:periodTime
                matE = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 0 0;0 0 0 0 0];
                matV = [dt*cos(rob_perState(3)) 0;dt*sin(rob_perState(3)) 0;0 dt;1 0;0 1];
                % 求解矩阵5*5*5*1+5*2*2*1 = 5*1向量表征机器人假设的状态
                % 该模型公式参考模型1，其中dt即△t，利用微分思想即dt是一个很小的数这里均取0.1
                rob_perState = matE*rob_perState+matV*[temp_v;temp_w];
            end
            
            %% 2.3 计算不同条件下的方位角评价函数,此时机器人状态是在预测时间后的假设状态
            % ①方向角评价函数是用来评价机器人在当前设定的采样速度下，
            % ②达到模拟轨迹末端时的朝向和目标之间的角度差距
            goalTheta=atan2(goalCoord(2)-rob_perState(2),goalCoord(1)-rob_perState(1));% 目标点的方位的角度
            evalTheta = abs(rob_perState(3)-goalTheta)/pi*180;
            heading = 180 - evalTheta;
            
            %% 2.4 计算不同条件下的空隙评价函数
            % ①空隙评价函数代表机器人在“当前轨迹上”与最近的障碍物之间的距离
            % ②如果在这条轨迹上没有障碍物，那就将其设定一个常数
            % ③障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
            dist = inf;
            for i=1:length(dy_obsCoord(:,1))
                % 利用二范数求距离
                disttmp=norm(dy_obsCoord(i,:)-rob_perState(1:2)')-obstacleR;
                % 保证dist是最近的障碍物距离
                if dist>disttmp
                    dist=disttmp;
                end
            end
            
            for (i=1:length(sta_obsCoord(:,1)))
                disttmp = norm(sta_obsCoord(i,:) - rob_perState(1:2)')-obstacleR;
                if(dist>disttmp)
                    dist=disttmp;
                end
            end
            
%             % 限定距离评价函数不能太大,同时对于没有障碍物的距离函数设置为2倍包容半径
%             if dist>=2*obstacleR
%                 dist=8*obstacleR;
%             end
            
            %% 2.7 计算不同条件下的距离轨迹最近的评价函数（自己添加的）
            dist_path = inf;
            for(i=1:length(path_listCoord(:,1)))
                dist_path_tmp = norm(path_listCoord(i,:) - rob_perState(1:2)')
                if dist_path > dist_path_tmp
                    dist_path = dist_path_tmp;
                end
            end
            
            dist_path = 1.2 - dist_path;
            
            
%             if dist_path>=3*obstacleR
%                 dist_path=3*obstacleR;
%             end

            
            %% 2.5 速度评价函数
            % 评价当前轨迹的速度值大小。速度越大，评分越高
            velocity = temp_v;

            %% 2.6 利用制动距离限定速度是在有效的
            % 制动距离的计算，保证所选的速度和加速度不会发生碰撞，参考了博客
            stopDist = temp_v*temp_v/(2*maxVelAcc);
            
            % 将有效的速度和角速度存入评价总的评价函数
            if dist>stopDist
                evalDB=[evalDB;[temp_v temp_w heading dist velocity 0 dist_path]];
            end
            
            
        end
    end
    
    
    %% 3、对当前状态所有假设的速度加速度组合的评价函数正则化，选取合适的加速度和速度作为下一状态
    % 如果评价函数为空则使得机器人停止，即evalDB全0
    if isempty(evalDB)
        evalDB = [0 0 0 0 0 0 0];
    end
    
    % 将评价函数进行正则化
    if sum(evalDB(:,3))~=0
        evalDB(:,3)=evalDB(:,3)/sum(evalDB(:,3));
    end
    if sum(evalDB(:,4))~=0
        evalDB(:,4)=evalDB(:,4)/sum(evalDB(:,4));
    end
    if sum(evalDB(:,5))~=0
        evalDB(:,5)=evalDB(:,5)/sum(evalDB(:,5));
    end
    if sum(evalDB(:,7)) ~= 0
        evalDB(:,7)=evalDB(:,7)/sum(evalDB(:,7));
    end
    
    % 最终评价函数的计算
    for i=1:length(evalDB(:,1))
        evalDB(i,6)=alpha*evalDB(i,3)+beta*evalDB(i,4)+gama*evalDB(i,5)+path_cofe*evalDB(i,7);
    end
    
    [~,ind]=max(evalDB(:,6));         % 选取出最优评价函数的索引
    nextVR=evalDB(ind,1:2)';          % 机器人下一速度和角速度即为该评价函数对应的速度和角速度
    
    %% 4、选择好角速度线速度更新机器人下一状态
    matE = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 0 0;0 0 0 0 0];
    matV = [dt*cos(rob_perState(3)) 0;dt*sin(rob_perState(3)) 0;0 dt;1 0;0 1];
    robot_NextState = matE*[robotXY(1),robotXY(2),robotT,robotV,robotW]'+matV*nextVR;
    
    % 更新状态开启下一轮DWA算法求解
    robotXY(1) = robot_NextState(1); robotXY(2) = robot_NextState(2);
    robotT = robot_NextState(3); robotV = robot_NextState(4);
    robotW = robot_NextState(5);
    
    % 将路径存放到路径矩阵，主要是为了绘图
    path = [path;[robotXY(1),robotXY(2)]];
    % 将速度控制指令存下来
    vel_cmd = [vel_cmd;[nextVR(1),nextVR(2)]];
    
    %% 5、绘制图像
    field(dy_obsIndex) = 1;
    dy_obsSub = coord2sub(dy_obsCoord);
    dy_obsR = dy_obsSub(:,1);dy_obsC = dy_obsSub(:,2);
    dy_obsIndex = sub2ind([rows,cols],dy_obsR,dy_obsC);
    field(dy_obsIndex) = 3;
    image(1.5,1.5,field);
    scatter(robotXY(1)+0.5,robotXY(2)+0.5,'r','LineWidth',1.5);     % 绘制机器人，红色圆圈表示
    plot(path(:,1)+0.5,path(:,2)+0.5,'-b');                                 % 绘制路径
    drawnow;
    
%     %% 6、将障碍物位置更新实现障碍物也在移动，对于静态障碍物删除从此往下代码
%     if mod(step,20) == 0
%         movpos = [0,1; 0,-1; -1,0; 1,0];                 % 对应上下左右四个方向
%         for i = 1:length(dy_obsCoord(:,1))
%             temp_obs = dy_obsCoord(i,:);
%             temp_pos = randi(4);
%             
%             % 移动范围限制在地图区间里
%             if dy_obsCoord(i,1) + movpos(temp_pos,1) > 0 && dy_obsCoord(i,1) + movpos(temp_pos,1) < cols
%                 if dy_obsCoord(i,2) + movpos(temp_pos,2) > 0 && dy_obsCoord(i,2) + movpos(temp_pos,2) < rows
%                     dy_obsCoord(i,1) = dy_obsCoord(i,1) + movpos(temp_pos,1);
%                     dy_obsCoord(i,2) = dy_obsCoord(i,2) + movpos(temp_pos,2);
%                 end
%             end
%         end
%     end
    step = step + 1;
end

figure(2);
plot(vel_cmd);
