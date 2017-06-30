function ptsBox = getPtsInBox(pts,box)
    flag = zeros(size(box,1),size(pts,1));
    for i = 1:size(box,1)
        condn1 = (box(i,1) <= pts(:,i));
        condn2 = (pts(:,i) <= box(i,2));
        flag(i,:) = condn1 & condn2;
    end
    flag = sum(flag,1);
    flag = (flag == size(box,1));
    ptsBox = pts(flag,:);
end