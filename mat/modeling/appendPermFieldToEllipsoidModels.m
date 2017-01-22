function ellipsoidModels = appendPermFieldToEllipsoidModels(ellipsoidModels,permVec)
    %APPENDPERMFIELDTOELLIPSOIDMODELS
    %
    % ellipsoidModels = APPENDPERMFIELDTOELLIPSOIDMODELS(ellipsoidModels,permVec)
    %
    % ellipsoidModels - nEllipsoids length struct array.
    % permVec         - nEllipsoids length vector.
    %
    % ellipsoidModels - nEllipsoids length struct array.

    nEllipsoids = length(ellipsoidModels);
    for i = 1:nEllipsoids
        ellipsoidModels(i).perm = permVec(i);
    end
end