function obb = calcObb(pts)
    %CALCOBB
    %
    % obb = CALCOBB(pts)
    %
    % pts - [nPts,3] array.
    %
    % obb - struct. fields ('center1','ax1','ax2','extents').
    
    ptsXy = pts(:,1:2);
    covMat = cov(ptsXy);
    [V,~] = eig(covMat);
    centroid = mean(pts,1);
    obb.center = centroid;
    % assigning larger eigenvalue to axis 1
    obb.ax1 = -V(:,2);
    obb.ax2 = -V(:,1);
    nPts = size(pts,1);
    obb.extents = zeros(3,2);
    lowLim = 1e-2;
    highLim = 1-lowLim;
    for i = 1:2
        vec = zeros(1,nPts);
        if i == 1
            dirn = obb.ax1;
        else
            dirn = obb.ax2;
        end
        for j = 1:nPts
            pt = pts(j,1:2)-obb.center(1:2);
            vec(j) = dot(dirn,pt);
        end
        obb.extents(i,1) = quantile(vec,lowLim);
        obb.extents(i,2) = quantile(vec,highLim);
    end
    
    for j = 1:nPts
        vec(j) = pts(j,3)-obb.center(3);
    end
    obb.extents(3,1) = quantile(vec,lowLim);
    obb.extents(3,2) = quantile(vec,highLim);
end
