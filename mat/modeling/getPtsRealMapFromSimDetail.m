function [ptsReal,ptsToRayOriginMap] = getPtsRealMapFromSimDetail(ptsRealCell)
    %GETPTSREALMAPFROMSIMDETAIL
    %
    % [ptsReal,ptsToRayOriginMap] = GETPTSREALMAPFROMSIMDETAIL(ptsRealCell)
    %
    % ptsRealCell       -
    %
    % ptsReal           -
    % ptsToRayOriginMap -
    
    ptsReal = [];
    ptsToRayOriginMap = [];
    for i = 1:length(ptsRealCell)
        ptsReal = [ptsReal; ptsRealCell{i}];
        ptsToRayOriginMap = [ptsToRayOriginMap ones(1,size(ptsRealCell{i},1))*i];
    end
end