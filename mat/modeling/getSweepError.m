function [errVec,missVec,detailIds] = getSweepError(simDetail,ptsGroundRef)
    [membershipIds,ptsReal,ptsSim,hitFlag] = unrollSimDetail(simDetail);
    
    thresh = 5e-3;
    [ids,dists] = knnsearch(ptsGroundRef,ptsReal);
    nonGroundFlag = (dists > thresh);
    nonGroundFlag = flipVecToRow(nonGroundFlag);
    
    rayOrigins = simDetail.rayOrigins;
    nDetail = size(rayOrigins,1);
    detailIds = 1:100:nDetail;
    [missVec,errVec] = deal(zeros(1,length(detailIds)));
    
    for i = 1:length(detailIds)
        detailId = detailIds(i);
        ptIds = find(membershipIds == detailId);
        % those which hit and are non ground
        flag = hitFlag(ptIds) & nonGroundFlag(ptIds);
        hitIds = ptIds(flag);
        
        pts1 = ptsReal(hitIds,:);
        pts2 = ptsSim(hitIds,:);
        [~,dists] = knnsearch(pts1,pts2);
        
        if ~isempty(dists)
            errVec(i) = mean(dists);
        end
        missVec(i) = sum(~hitFlag(ptIds) & nonGroundFlag(ptIds));
    end
    
end

