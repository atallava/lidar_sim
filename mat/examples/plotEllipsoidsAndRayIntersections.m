function hfig = plotEllipsoidsAndRayIntersections(meanCell,covMatCell,ptsRay,intersectionFlag)
    nEllipses = length(meanCell);
    faceAlphaIntersect = 0.2;
    faceAlphaPass = 0;
    for i = 1:nEllipses
        if intersectionFlag(i)
            faceAlpha = faceAlphaIntersect;
        else
            continue;
        end
        thisMean = meanCell{i};
        thisCovMat = covMatCell{i};
        
        [xEll,yEll,zEll] = genSurfXyzEllipse(thisCovMat,thisMean,3.5^2);
        surf(xEll,yEll,zEll,'facecolor','r','facealpha',faceAlpha,'meshstyle','none');
        hold on;
    end
    axis equal;
    plot3(ptsRay(:,1),ptsRay(:,2),ptsRay(:,3),'g');
    hfig = gcf;
end