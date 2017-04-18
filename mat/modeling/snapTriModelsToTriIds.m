function triModelsSnapped = snapTriModelsToTriIds(triModels,triIds)
    if nargin < 2
        triIds = 1:size(triModels.tri,1);
    end
    
    tris = triModels.tri(triIds,:);
    ptsFitIds = unique(tris(:));
    
    u = tris(:);
    u = flipVecToRow(u);
    v = ptsFitIds;
    v = flipVecToColumn(v);
    U = repmat(u,length(v),1);
    V = repmat(v,1,length(u));
    flagMat = (U == V);
    triSnappedVec = find(flagMat);
    triSnappedVec = mod(triSnappedVec,length(ptsFitIds));
    triSnappedVec(triSnappedVec == 0) = length(ptsFitIds);
    triSnapped = reshape(triSnappedVec,length(triSnappedVec)/3,3);
    
    triModelsSnapped.tri = triSnapped;
    triModelsSnapped.ptsFit = triModels.ptsFit(ptsFitIds,:);
    triModelsSnapped.hitProbVec = triModels.hitProbVec(triIds);
    triModelsSnapped.rangeVar = triModels.rangeVar;
end