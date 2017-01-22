function d = calcMahalanobisDist(mu,covMat,pt)
    %CALCMAHALANOBISDIST
    %
    % d = CALCMAHALANOBISDIST(mu,covMat,pt)
    %
    % mu     - vector.
    % covMat - covariance matrix.
    % pt     - vector.
    %
    % d      - scalar.
    
    mu = flipVecToColumn(mu);
    pt = flipVecToColumn(pt);
    d = sqrt((pt-mu)'*(covMat\(pt-mu)));
end