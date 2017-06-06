function [rayOriginId,rayDirnId] = findMatchInSimDetail(ptsRealCell,pt)
    %FINDMATCHINSIMDETAIL
    %
    % [rayOriginId,rayDirnId] = FINDMATCHINSIMDETAIL(ptsRealCell,pt)
    %
    % ptsRealCell -
    % pt          -
    %
    % rayOriginId -
    % rayDirnId   -
    
    [ptsReal,ptsToRayOriginMap] = getPtsRealMapFromSimDetail(ptsRealCell);
    minId = knnsearch(ptsReal,pt);
    rayOriginId = ptsToRayOriginMap(minId);
    vec = find(ptsToRayOriginMap == rayOriginId);
    rayDirnId = (minId - vec(1))+1;
end