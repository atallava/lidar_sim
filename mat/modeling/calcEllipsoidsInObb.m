function obbEllipsoids = calcEllipsoidsInObb(ellipsoidModels,obb)
    %CALCELLIPSOIDSINOBB
    %
    % obbEllipsoids = CALCELLIPSOIDSINOBB(ellipsoidModels,obb)
    %
    % ellipsoidModels -
    % obb             -
    %
    % obbEllipsoids   -
    
    % get ellipsoids in some radius of obb
    searchRadiusPadding = 1;
    searchRadius = calcObbMaxExtent(obb)+searchRadiusPadding;
    ellipsoidCenters = getEllipsoidCenters(ellipsoidModels);
    candidateIds = rangesearch(ellipsoidCenters,obb.center,searchRadius);
    candidateIds = candidateIds{1};
    
    % retain those in the obb
    flag = checkPtsInObb(ellipsoidCenters(candidateIds,:),obb);
    chosenIds = candidateIds(flag);
    
    obbEllipsoids = ellipsoidModels(chosenIds);    
end