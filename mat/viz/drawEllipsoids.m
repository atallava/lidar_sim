function drawEllipsoids(hfig,ellipsoidModels)
%DRAWELLIPSOIDS
%
% DRAWELLIPSOIDS(hfig,ellipsoidModels)
%
% hfig            -
% ellipsoidModels -

figure(hfig); hold on;
ellipsoidUniformAlpha = false;
nEllipses = length(ellipsoidModels);
for i = 1:nEllipses
    thisMean = ellipsoidModels(i).mu;
    thisCovMat = ellipsoidModels(i).covMat;
    thisHitProb = ellipsoidModels(i).hitProb;
    
    [xEll,yEll,zEll] = genSurfXyzEllipse(thisCovMat,thisMean);
    
    if ellipsoidUniformAlpha
        thisAlpha = 0.2;
    else
        thisAlpha = mapHitProbToAlpha(thisHitProb);
    end
    
    surf(xEll,yEll,zEll,'facecolor','g','meshstyle','none', ...
        'facealpha',thisAlpha);
end
end