function [ptsReal,ptsSim] = getNonGroundPcdsFromSimDetail(simDetail,ptsGroundRef)
    ptsRealCell = simDetail.ptsRealCell;
    ptsRealCell = flipVecToColumn(ptsRealCell);
    ptsReal = cell2mat(ptsRealCell);
    
    thresh = 5e-3;
    [ids,dists] = knnsearch(ptsGroundRef,ptsReal);
    nonGroundFlag = (dists > thresh);
    ptsReal = ptsReal(nonGroundFlag,:);
    
    ptsSimCell = simDetail.ptsSimCell;
    ptsSimCell = flipVecToColumn(ptsSimCell);
    ptsSim = cell2mat(ptsSimCell);
    ptsSim = ptsSim(nonGroundFlag,:);
    
    hitFlagCell = simDetail.hitFlagCell;
    hitFlagCell = flipVecToRow(hitFlagCell);
    hitFlag = cell2mat(hitFlagCell);
    hitFlag = hitFlag(nonGroundFlag);
    hitFlag = logical(hitFlag);
    
    ptsSim = ptsSim(hitFlag,:);
end