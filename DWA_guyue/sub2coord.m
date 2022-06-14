function coord = sub2coord(sub)
%SUB2COORD 将行列式下标装换为坐标格式，此时的坐标格式和原本认知坐标方向也不一致（如下所示）
%        1 2 3 4 5 6 7 .... X坐标
%      1|——————————>
%      2|
%      3|
%      4|
%      5|
%  Y坐标\/

    [l,w] = size(sub);
    % 长度l=2表示sub为2*n矩阵
    if l == 2
        coord(1,:) = sub(2,:);
        coord(2,:) = sub(1,:);
    end
    
    if w == 2
        coord(:,1) = sub(:,2);
        coord(:,2) = sub(:,1);
    end

end
