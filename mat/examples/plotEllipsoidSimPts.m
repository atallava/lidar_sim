function hfig = plotEllipsoidSimPts(rayOrigin,rayDirns,meanCell,covMatCell,simPts,hitFlag)
    
    % ellipses
    nEllipses = length(meanCell);
    for i = 1:nEllipses
        thisMean = meanCell{i};
        thisCovMat = covMatCell{i};
        
        [xEll,yEll,zEll] = genSurfXyzEllipse(thisCovMat,thisMean);
        surf(xEll,yEll,zEll,'facecolor','r','facealpha',0.2,'meshstyle','none');
        hold on;
    end
    axis equal;

    % hit rays
    nRays = size(rayDirns,1);
    for i = 1:nRays
        if hitFlag(i)
            pts = genPtsRay(rayOrigin,rayDirns(i,:),10);
            plot3(pts(:,1),pts(:,2),pts(:,3),'g--');
        end
    end
    
    hitFlag = logical(hitFlag);
    % sim pts
    plot3(simPts(hitFlag,1),simPts(hitFlag,2),simPts(hitFlag,3),'go');

    hfig = gcf;
    
end