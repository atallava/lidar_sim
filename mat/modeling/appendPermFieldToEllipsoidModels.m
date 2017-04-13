function ellipsoidModels = appendHitProbFieldToEllipsoidModels(ellipsoidModels,hitProbVec)
    %APPENDHITPROBFIELDTOELLIPSOIDMODELS
    %
    % ellipsoidModels = APPENDHITPROBFIELDTOELLIPSOIDMODELS(ellipsoidModels,hitProbVec)
    %
    % ellipsoidModels - nEllipsoids length struct array.
    % hitProbVec         - nEllipsoids length vector.
    %
    % ellipsoidModels - nEllipsoids length struct array.

    nEllipsoids = length(ellipsoidModels);
    for i = 1:nEllipsoids
        ellipsoidModels(i).hitProb = hitProbVec(i);
    end
end
