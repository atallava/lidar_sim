function ids = findNbrTriIds(triModels,pts,maxDist)
%FINDNBRTRIIDS
%
% ids = FINDNBRTRIIDS(triModels,pt,maxDist)
%
% triModels -
% pt        -
% maxDist   -
%
% ids       -

%     D = pdist2(triModels.ptsFit,pts);
%     D = min(D,[],2);
%     ptsFitNbrIds = find(D < maxDist);

searchRes = rangesearch(triModels.ptsFit,pts,maxDist);
ptsFitNbrIds = cell2mat(searchRes');
ptsFitNbrIds = unique(ptsFitNbrIds);

vertexFlags = zeros(size(triModels.tri,1),3);
for i = 1:3
    vertexFlags(:,i) = ismember(triModels.tri(:,i),ptsFitNbrIds);
end
flag = logical(sum(vertexFlags,2));

ids = find(flag);
end