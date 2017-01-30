function ptsFit = restrictFitToObsProjn(pts,ptsFit,maxDistToProjn)
%RESTRICTFITTOOBSPROJN Retain those ptsFit whose xy is close to xy of pts.
% 
% ptsFit = RESTRICTFITTOOBSPROJN(pts,ptsFit,maxDistToProjn)
% 
% pts            - [nPts,3] array.
% ptsFit         - [nPtsFit,3] array.
% maxDistToProjn - scalar.
% 
% ptsFit         - [nPtsFit,3] array.

xy = pts(:,1:2);
xyFit = ptsFit(:,1:2);
fprintf('n pts: %d\n', size(xy,1));
fprintf('n nodes: %d\n', size(xyFit,1));

D = pdist2(xyFit,xy);
DMin = min(D,[],2);

flag = (DMin <= maxDistToProjn);
ptsFit = ptsFit(flag,:);
end
