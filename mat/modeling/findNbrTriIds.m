function ids = findNbrTriIds(triModels,pt,maxDist)
    %FINDNBRTRIIDS
    %
    % ids = FINDNBRTRIIDS(triModels,pt,maxDist)
    %
    % triModels -
    % pt        -
    % maxDist   -
    %
    % ids       -
    
    D = pdist2(triModels.ptsFit,pt);
    ptsFitNbrIds = find(D < maxDist);
    
    vertexFlags = zeros(size(triModels.tri,1),3);
    for i = 1:3
        vertexFlags(:,i) = equalsVec(triModels.tri(:,i),ptsFitNbrIds);
    end
    flag = logical(sum(vertexFlags,2));
    ids = find(flag);
end