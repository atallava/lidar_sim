function calcRangeError(relPathSimDetail)
    [rayOrigins,ptsRealCell,ptsSimCell,hitFlagCell] = loadSimDetail(relPathSimDetail);
    
    nOrigins = size(rayOrigins,1);
    [errVec, errVecOnlyHits, ...
        nRealPtsVec, nMissesVec] = deal(zeros(1,nOrigins));
    
    for i = 1:nOrigins
        rayOrigin = rayOrigins(i,:);
        ptsReal = ptsRealCell{i};
        ptsSim = ptsSimCell{i};
        hitFlag = hitFlagCell{i};
        
        nRealPts = size(ptsReal,1);
        nRealPtsVec(i) = nRealPts;
        
        realRanges = bsxfun(@minus,ptsReal,rayOrigin);
        realRanges = sqrt(sum(realRanges.^2,2));
        
        simRanges = bsxfun(@minus,ptsSim,rayOrigin);
        simRanges = sqrt(sum(simRanges.^2,2));
        
        err = (simRanges-realRanges).^2;
        
        errVec(i) = mean(err);
        errVecOnlyHits(i) = mean(err(hitFlag));
        nMissesVec(i) = sum(~hitFlag);
    end
    
    fracnMissesVec = nMissesVec./nRealPtsVec;
    errMean = mean(errVec);
    errStd = std(errVec);
    fprintf('err. mean: %.2f. std: %.2f\n',errMean,errStd);
    
    errOnlyHitsMean = mean(errVecOnlyHits);
    errOnlyHitsStd = std(errVecOnlyHits);
    fprintf('err only hits. mean: %.2f. std: %.2f\n',errOnlyHitsMean,errOnlyHitsStd);
    
    fprintf('overall fracn misses: %.2f\n',sum(nMissesVec)/sum(nRealPtsVec));
end