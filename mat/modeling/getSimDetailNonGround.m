function simDetailNonGround = getSimDetailNonGround(simDetail,ptsGround)
    nDetail = size(simDetail.rayOrigins,1);
    thresh = 5e-3;
    [ptsRealNonGroundCell,ptsSimNonGroundCell,hitFlagNonGroundCell] = ...
        deal(cell(1,nDetail));
    for i = 1:nDetail
        ptsReal = simDetail.ptsRealCell{i};
        [ids,dists] = knnsearch(ptsGround,ptsReal);
        flag = (dists > thresh);
        
        ptsRealNonGround = ptsReal(flag,:);
        ptsSim = simDetail.ptsSimCell{i};
        ptsSimNonGround = ptsSim(flag,:);
        hitFlag = simDetail.hitFlagCell{i};
        hitFlagNonGround = hitFlag(flag);
        
        ptsRealNonGroundCell{i} = ptsRealNonGround;
        ptsSimNonGroundCell{i} = ptsSimNonGround;
        hitFlagNonGroundCell{i} = hitFlagNonGround;
    end
    
    simDetailNonGround.rayOrigins = simDetail.rayOrigins;
    simDetailNonGround.ptsRealCell = ptsRealNonGroundCell;
    simDetailNonGround.ptsSimCell = ptsSimNonGroundCell;
    simDetailNonGround.hitFlagCell = hitFlagNonGroundCell;
end