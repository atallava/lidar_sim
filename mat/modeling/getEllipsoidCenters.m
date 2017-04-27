function centers = getEllipsoidCenters(ellipsoidModels)
    %GETELLIPSOIDCENTERS
    %
    % centers = GETELLIPSOIDCENTERS(ellipsoidModels)
    %
    % ellipsoidModels - struct array.
    %
    % centers         - [nEllipsoids,3] array.
    
    muVec = [ellipsoidModels.mu];
    muMat = reshape(muVec,3,length(muVec)/3);
    centers = muMat';
end