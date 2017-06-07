function obbTriModels = snapTriModelsToObb(triModels,obb)
    %SNAPTRIMODELSTOOBB
    %
    % obbTriModels = SNAPTRIMODELSTOOBB(triModels,obb)
    %
    % triModels    -
    % obb          -
    %
    % obbTriModels -
    
    flag = checkPtsInObb(triModels.ptsFit,obb);
    ptIds = find(flag);
    triIds = findTriIdsForPtIds(triModels,ptIds);
    obbTriModels = snapTriModelsToTriIds(triModels,triIds);
end