function ptsFit = restrictFitToObsProjn(pts,ptsFit)
xy = pts(:,1:2);
xyFit = ptsFit(:,1:2);
D = pdist2(xyFit,xy);
DMin = min(D,[],2);

% TODO: put this parameter in a file
maxDistToProjn = 1;

flag = (DMin <= maxDistToProjn);
ptsFit = ptsFit(flag,:);
end