function ids = findTriIdsForPtIds(triModels,ptIds)
    vertexFlags = zeros(size(triModels.tri,1),3);
    for i = 1:3
        vertexFlags(:,i) = equalsVec(triModels.tri(:,i),ptIds);
    end
    flag = logical(sum(vertexFlags,2));
    ids = find(flag);
end