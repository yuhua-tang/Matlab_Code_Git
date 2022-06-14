clear;
% 机器人现在状态
robotX = 1;                         % 机器人当前X位置
robotY = 1;                         % 机器人当前Y位置
robotT = pi/2;                      % 机器人当前方向角度(0->2pi)
robotV = 0;                         % 机器人当前速度
robotW = 0;                         % 机器人当前角速度

% 地图信息
goal=[9,9];                         % 目标点位置 [x(m),y(m)]
obstacle=[2,2;4,4;6,6;8,8];         % 障碍物位置列表 [x(m) y(m)]
obstacleR=0.6;                      % 冲突判定用的障碍物半径
dt = 0.1;                           % 时间[s]

% 机器人运动学模型
maxVel = 1.0;                       % 机器人最大速度m/s
maxRot = 20.0/180*pi;               % 机器人最大角速度rad/s
maxVelAcc = 0.2;                    % 机器人最大加速度m/ss
maxRotAcc = 50.0/180*pi;            % 机器人最大角加速度rad/ss


% 评价系数
alpha = 0.05;                       % 方位角评价系数α
beta = 0.2;                         % 空隙评价系数β
gama = 0.1;                         % 速度评价系数γ
periodTime = 3.0;                   % 预测处理时间,也就是绿色搜寻轨迹的长度

area=[0 10 0 10];                   % 模拟区域范围 [xmin xmax ymin ymax]
path = [];                          % 记录移动路径

%% 开始DWA算法求解
while true
    
    % 是否到达目的地，到达目的地则跳出循环
    if norm([robotX,robotY]-goal') < 0.5
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
            rob_perState = [robotX,robotY,robotT,robotV,robotW]';
            
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
            goalTheta=atan2(goal(2)-rob_perState(2),goal(1)-rob_perState(1));% 目标点的方位的角度
            evalTheta = abs(rob_perState(3)-goalTheta)/pi*180;
            heading = 180 - evalTheta;
            
            %% 2.4 计算不同条件下的空隙评价函数
            % ①空隙评价函数代表机器人在“当前轨迹上”与最近的障碍物之间的距离
            % ②如果在这条轨迹上没有障碍物，那就将其设定一个常数
            % ③障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
            dist = inf;
            for i=1:length(obstacle(:,1))
                % 利用二范数求距离
                disttmp=norm(obstacle(i,:)-rob_perState(1:2)')-obstacleR;
                % 保证dist是最近的障碍物距离
                if dist>disttmp
                    dist=disttmp;
                end
            end
            
            % 限定距离评价函数不能太大,同时对于没有障碍物的距离函数设置为2倍包容半径
            if dist>=2*obstacleR
                dist=2*obstacleR;
            end

            
            %% 2.5 速度评价函数
            % 评价当前轨迹的速度值大小。速度越大，评分越高
            velocity = temp_v;

            %% 2.6 利用制动距离限定速度是在有效的
            % 制动距离的计算，保证所选的速度和加速度不会发生碰撞，参考了博客
            stopDist = temp_v*temp_v/(2*maxVelAcc);
            
            % 将有效的速度和角速度存入评价总的评价函数
            if dist>stopDist
                evalDB=[evalDB;[temp_v temp_w heading dist velocity 0]];
            end
        end
    end
    
    
    %% 3、对当前状态所有假设的速度加速度组合的评价函数正则化，选取合适的加速度和速度作为下一状态
    % 如果评价函数为空则使得机器人停止，即evalDB全0
    if isempty(evalDB)
        evalDB = [0 0 0 0 0 0];
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
    
    % 最终评价函数的计算
    for i=1:length(evalDB(:,1))
        evalDB(i,6)=alpha*evalDB(i,3)+beta*evalDB(i,4)+gama*evalDB(i,5);
    end
    
    [~,ind]=max(evalDB(:,6));         % 选取出最优评价函数的索引
    nextVR=evalDB(ind,1:2)';          % 机器人下一速度和角速度即为该评价函数对应的速度和角速度
    
    %% 4、选择好角速度线速度更新机器人下一状态
    matE = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 0 0;0 0 0 0 0];
    matV = [dt*cos(rob_perState(3)) 0;dt*sin(rob_perState(3)) 0;0 dt;1 0;0 1];
    robot_NextState = matE*[robotX,robotY,robotT,robotV,robotW]'+matV*nextVR;
    
    % 更新状态开启下一轮DWA算法求解
    robotX = robot_NextState(1); robotY = robot_NextState(2);
    robotT = robot_NextState(3); robotV = robot_NextState(4);
    robotW = robot_NextState(5);
    
    % 将路径存放到路径矩阵，主要是为了绘图
    path = [path;[robotX,robotY]];
    
    %% 5、绘制图像
    hold off;
    scatter(robotX,robotY,'r','LineWidth',1.5);hold on;     % 绘制机器人，红色圆圈表示
    plot(goal(1),goal(2),'*r');hold on;                     % 绘制地图终点
    scatter(obstacle(:,1),obstacle(:,2),200,'k');hold on;   % 绘制障碍物
    plot(path(:,1),path(:,2),'-b');hold on;                 % 绘制路径
    axis(area);
    grid on;
    drawnow;
    
    %% 6、将障碍物位置更新实现障碍物也在移动，对于静态障碍物删除从此往下代码
%     movpos = [0,0.2; 0,-0.2; -0.2,0; 0.2,0];                 % 对应上下左右四个方向
%     for i = 1:length(obstacle(:,1))
%         temp_obs = obstacle(i,:);
%         temp_pos = randi(4);
%         
%         % 移动范围限制在地图区间里
%         if obstacle(i,1) + movpos(temp_pos,1) > 0 && obstacle(i,1) + movpos(temp_pos,1) < 10
%             if obstacle(i,2) + movpos(temp_pos,2) > 0 && obstacle(i,2) + movpos(temp_pos,2) < 10
%                 obstacle(i,1) = obstacle(i,1) + movpos(temp_pos,1);
%                 obstacle(i,2) = obstacle(i,2) + movpos(temp_pos,2);
%             end
%         end
%     end
    
    
end