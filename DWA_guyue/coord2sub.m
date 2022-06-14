function sub = coord2sub(coord)
%COORD2SUB 将坐标转换为矩阵行列格式，坐标格式为下图所示
%        1 2 3 4 5 6 7 .... X坐标
%      1|——————————>
%      2|
%      3|
%      4|
%      5|
%  Y坐标\/

    [l,w] = size(coord);
    % 长度l=2表示sub为2*n矩阵
    if l == 2
        sub(1,:) = coord(2,:);
        sub(2,:) = coord(1,:);
    end
    
    if w == 2
        sub(:,1) = coord(:,2);
        sub(:,2) = coord(:,1);
    end
end