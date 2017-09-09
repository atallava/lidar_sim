function ptsNonGround = removeGroundPtsFromCloud(ptsQuery,ptsGroundRef)
    thresh = 5e-3;
    [ids,dists] = knnsearch(ptsGroundRef,ptsQuery);
    flag = (dists > thresh);
    ptsNonGround = ptsQuery(flag,:);
end