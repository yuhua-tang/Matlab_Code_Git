function PlotGridArea(X_Length,Y_Length,Start_Node,Target_Node,Obs_Node_List,OpenList,OpenList_Num,CloseList,CloseList_Num)
figure(1);
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
%CloseList
for i = 1:CloseList_Num
    Close_Node = CloseList(i);
    if Close_Node.PositionX == Start_Node(1) && Close_Node.PositionY == Start_Node(2)%如果是Start_Node就不当作CloseNode画
        continue;
    elseif Close_Node.PositionX == Target_Node(1) && Close_Node.PositionY == Target_Node(2)%如果是Target_Node就不当作CloseNode画
        continue;
    end
    Close_Node_X = Close_Node.PositionX;
    Close_Node_Y = Close_Node.PositionY;
    fill([Close_Node_X,Close_Node_X,Close_Node_X-1,Close_Node_X-1,Close_Node_X],[Close_Node_Y,Close_Node_Y-1,Close_Node_Y-1,Close_Node_Y,Close_Node_Y],'blue'); 
end
%OpenList
for i = 1:OpenList_Num
    Open_Node = OpenList(i);
    if Open_Node.PositionX == Start_Node(1) && Open_Node.PositionY == Start_Node(2)%如果是Start_Node就不当作OpenNode画
        continue;
    elseif Open_Node.PositionX == Target_Node(1) && Open_Node.PositionY == Target_Node(2)%如果是Target_Node就不当作OpenNode画
        continue;
    end
    Open_Node_X = Open_Node.PositionX;
    Open_Node_Y = Open_Node.PositionY;
    fill([Open_Node_X,Open_Node_X,Open_Node_X-1,Open_Node_X-1,Open_Node_X],[Open_Node_Y,Open_Node_Y-1,Open_Node_Y-1,Open_Node_Y,Open_Node_Y],'red'); 
end

end