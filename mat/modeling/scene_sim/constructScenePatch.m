function [patchPtsCell,patchEllipsoidsCell] = constructScenePatch(patchAnnotation,primitives)
%CONSTRUCTSCENEPATCH
%
% [patchPtsCell,patchEllipsoidsCell] = CONSTRUCTSCENEPATCH(patchAnnotation,primitives)
%
% patchAnnotation     -
% primitives          -
%
% patchPtsCell        -
% patchEllipsoidsCell -

nPatchCells = length(patchAnnotation.T_cells_to_world);
[patchPtsCell,patchEllipsoidsCell] = deal(cell(1,nPatchCells));
for i = 1:nPatchCells
    % this cell data
    thisCellObb_world = patchAnnotation.cellObbs_world{i};
    thisCellT = patchAnnotation.T_cells_to_world{i};
    thisCellAnnotation = struct('objectClass',patchAnnotation.objectClass, ....
        'objectObb_world',thisCellObb_world,'T_object_to_world',thisCellT);
    
    % construct this cell
    [thisCellPts,thisCellEllipsoids] = constructSceneObject(thisCellAnnotation,primitives);
    
    % add to list
    patchPtsCell{i} = thisCellPts;
    patchEllipsoidsCell{i} = thisCellEllipsoids;
end
end