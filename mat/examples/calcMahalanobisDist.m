function d = calcMahalanobisDist(mu,covMat,pt)
    mu = flipVecToColumn(mu);
    pt = flipVecToColumn(pt);
    d = sqrt((pt-mu)'*(covMat\(pt-mu)));
end