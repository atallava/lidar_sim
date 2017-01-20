function hfig = plotEllipsoidsAndRay(meanCell,covMatCell,ptsRay)
    nEllipses = length(meanCell);
    for i = 1:nEllipses
        thisMean = meanCell{i};
        thisCovMat = covMatCell{i};
        
        [xEll,yEll,zEll] = genSurfXyzEllipse(thisCovMat,thisMean);
        surf(xEll,yEll,zEll,'facecolor','r','facealpha',0.2,'meshstyle','none');
        hold on;
    end
    axis equal;
    plot3(ptsRay(:,1),ptsRay(:,2),ptsRay(:,3),'g');
    hfig = gcf;
end