function flag = checkPtsInObb(pts,obb)
    %CHECKPTSINOBB
    %
    % flag = CHECKPTSINOBB(pts,obb)
    %
    % pts  -
    % obb  -
    %
    % flag -
    
    ptsCentered = bsxfun(@minus,pts,obb.center);
    
    projns = zeros(size(pts));
    tmpMat = bsxfun(@times,ptsCentered(:,1:2),flipVecToRow(obb.ax1));
    projns(:,1) = sum(tmpMat,2);
    tmpMat = bsxfun(@times,ptsCentered(:,1:2),flipVecToRow(obb.ax2));
    projns(:,2) = sum(tmpMat,2);
    projns(:,3) = ptsCentered(:,3);
    
    condns = zeros(size(pts));
    for i = 1:3
        condnLow = obb.extents(i,1) <= projns(:,i);
        condnHigh = projns(:,i) <= obb.extents(i,2);
        condns(:,i) = (condnLow & condnHigh);
    end
    
    flag = (sum(condns,2) == 3);
end