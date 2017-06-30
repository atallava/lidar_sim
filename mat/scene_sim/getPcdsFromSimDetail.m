function [ptsReal,ptsSim] = getPcdsFromSimDetail(simDetail)
    ptsRealCell = simDetail.ptsRealCell;
    ptsRealCell = flipVecToColumn(ptsRealCell);
    ptsReal = cell2mat(ptsRealCell);
    
    ptsSimCell = simDetail.ptsSimCell;
    ptsSimCell = flipVecToColumn(ptsSimCell);
    ptsSim = cell2mat(ptsSimCell);
    
    hitFlagCell = simDetail.hitFlagCell;
    hitFlagCell = flipVecToRow(hitFlagCell);
    hitFlag = cell2mat(hitFlagCell);
    hitFlag = logical(hitFlag);
    
    ptsSim = ptsSim(hitFlag,:);
end