function obbTriModels = snapTriModelsToObb(triModels,obb)
    flag = checkPtsInObb(triModels.ptsFit,obb);
    ptIds = find(flag);
    triIds = findTriIdsForPtIds(triModels,ptIds);
    obbTriModels = snapTriModelsToTriIds(triModels,triIds);
end