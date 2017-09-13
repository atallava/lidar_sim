function triModelsSnapped = snapTriModelsToTriIds(triModels,triIds)
%SNAPTRIMODELSTOTRIIDS
%
% triModelsSnapped = SNAPTRIMODELSTOTRIIDS(triModels,triIds)
%
% triModels        -
% triIds           -
%
% triModelsSnapped -

if nargin < 2
    triIds = 1:size(triModels.tri,1);
end

tris = triModels.tri(triIds,:);
ptsFitIds = unique(tris(:));

% this approach retains triangles which are not in triIds, but whose
% vertices may contain a point included in triIds. this approach dies when
% the mesh is large
% example: 17026 tri, 28084 pts in tri, total 2545469 pts
% todo: i also forgot how this code runs
% u = tris(:);
% u = flipVecToRow(u);
% v = ptsFitIds;
% v = flipVecToColumn(v);
% U = repmat(u,length(v),1);
% V = repmat(v,1,length(u));
% flagMat = (U == V);
% triSnappedVec = find(flagMat);
% triSnappedVec = mod(triSnappedVec,length(ptsFitIds));
% triSnappedVec(triSnappedVec == 0) = length(ptsFitIds);
% triSnapped = reshape(triSnappedVec,length(triSnappedVec)/3,3);

% this one just prunes away all triangles except triIds
maxPtId = max(ptsFitIds);
nPtsFitIds = length(ptsFitIds);
helperVec = zeros(1,maxPtId); 
helperVec(ptsFitIds) = 1:nPtsFitIds; % to help reassign pt ids
triSnappedVec = helperVec(tris(:));
triSnapped = reshape(triSnappedVec,length(triSnappedVec)/3,3);

triModelsSnapped.tri = triSnapped;
triModelsSnapped.ptsFit = triModels.ptsFit(ptsFitIds,:);
triModelsSnapped.hitProbVec = triModels.hitProbVec(triIds);
triModelsSnapped.rangeVar = triModels.rangeVar;
end