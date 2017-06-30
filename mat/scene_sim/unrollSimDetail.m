function [membershipIds,ptsReal,ptsSim,hitFlag] = unrollSimDetail(simDetail)
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
    
    membershipIds = [];
    for i = 1:length(ptsRealCell)
        membershipIds = [membershipIds ones(1,size(ptsRealCell{i},1))*i];
    end
end