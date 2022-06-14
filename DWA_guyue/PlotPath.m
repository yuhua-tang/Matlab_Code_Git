%% 搜索出路径后，通过画图画出来
function PlotPath(X_Length,Y_Length,Start_Node,Target_Node,Obs_Node_List,Path_List)
figure(2);
% 画网格线
for x = 1:X_Length
    plot([x,x],[0,Y_Length],'black');
    hold on;
end
for y = 1:Y_Length
    plot([0,X_Length],[y,y],'black');
    hold on;
end
%坐标轴调整
axis equal;
axis([0,X_Length,0,Y_Length]);

%Start_Node
Start_Node_X = Start_Node(1);
Start_Node_Y = Start_Node(2);
fill([Start_Node_X,Start_Node_X,Start_Node_X-1,Start_Node_X-1,Start_Node_X],[Start_Node_Y,Start_Node_Y-1,Start_Node_Y-1,Start_Node_Y,Start_Node_Y],'yellow');
%Target_Node
Target_Node_X = Target_Node(1);
Target_Node_Y = Target_Node(2);
fill([Target_Node_X,Target_Node_X,Target_Node_X-1,Target_Node_X-1,Target_Node_X],[Target_Node_Y,Target_Node_Y-1,Target_Node_Y-1,Target_Node_Y,Target_Node_Y],'green');
%Obs_Node
for i = 1:length(Obs_Node_List)
    Obs_Node_X = Obs_Node_List(i,1);
    Obs_Node_Y = Obs_Node_List(i,2);
    fill([Obs_Node_X,Obs_Node_X,Obs_Node_X-1,Obs_Node_X-1,Obs_Node_X],[Obs_Node_Y,Obs_Node_Y-1,Obs_Node_Y-1,Obs_Node_Y,Obs_Node_Y],'black');    
end
%Path_List
plot(Path_List(:,1)-0.5,Path_List(:,2)-0.5,'*');
plot(Path_List(:,1)-0.5,Path_List(:,2)-0.5,'-');
%Cost_F
for i = 1:size(Path_List,1)
    PositionX = Path_List(i,1);
    PositionY = Path_List(i,2);
    Cost_F = Path_List(i,3);
    text(PositionX - 1,PositionY - 0.2,num2str(Cost_F));
end
end