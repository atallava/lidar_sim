function pts2 = applyTransf(pts1,T_1_to_2)
    if size(pts1,2) == 3
        hInput = false;
        pts1 = [pts1 ones(size(pts1,1),1)];
    else
        hInput = true;
    end
    pts1 = pts1';
    pts2 = T_1_to_2*pts1;
    pts2 = pts2';
    
    if ~hInput
        pts2(:,4) = [];
        return;
    else
        return;
    end
end