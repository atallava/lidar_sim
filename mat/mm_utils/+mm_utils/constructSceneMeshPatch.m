function patchTriModelsCell = constructSceneMeshPatch(patchAnnotation, primitives)
%CONSTRUCTSCENEMESHPATCH
%
% patchTriModelsCell = CONSTRUCTSCENEMESHPATCH(patchAnnotation, primitives)
%
% patchAnnotation    -
% primitives         -
%
% patchTriModelsCell -

nPatchCells = length(patchAnnotation.T_cells_to_world);
patchTriModelsCell = cell(1,nPatchCells);
for i = 1:nPatchCells
    % this cell data
    thisCellObb_world = patchAnnotation.cellObbs_world{i};
    thisCellT = patchAnnotation.T_cells_to_world{i};
    thisCellAnnotation = struct('objectClass',patchAnnotation.objectClass, ....
        'objectObb_world',thisCellObb_world,'T_object_to_world',thisCellT);
    
    % construct this cell
    thisCellTriModels = mm_utils.constructSceneMeshObject(thisCellAnnotation, primitives);
        
    % add to list
    patchTriModelsCell{i} = thisCellTriModels;
end
end