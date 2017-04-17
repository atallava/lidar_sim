function [rayOriginId,rayDirnId] = findMatchInSimDetail(ptsRealCell,pt)
    [ptsReal,ptsToRayOriginMap] = getPtsRealMapFromSimDetail(ptsRealCell);
    minId = knnsearch(ptsReal,pt);
    rayOriginId = ptsToRayOriginMap(minId);
    vec = find(ptsToRayOriginMap == rayOriginId);
    rayDirnId = (minId - vec(1))+1;
end