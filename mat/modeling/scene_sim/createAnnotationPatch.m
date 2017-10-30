function annotation = createAnnotationPatch(segmentClass,pts)
%CREATEANNOTATIONPATCH
%
% annotation = CREATEANNOTATIONPATCH(segmentClass,pts)
%
% segmentClass - scalar.
% pts          - [nPts,3] array. Assumed to be in world frame.
%
% annotation   - struct.

% get cell obbs
[cellObbs_world,cellPts_world] = calcPatchObbs(pts);
nCellsInPatch = length(cellObbs_world);
T_cells_to_world = cell(1,nCellsInPatch);
for j = 1:nCellsInPatch
    T_obb_to_world = getObbTransf(cellObbs_world{j});
    T_cells_to_world{j} = T_obb_to_world;
end

annotation.objectClass = segmentClass;
annotation.cellObbs_world = cellObbs_world;
annotation.T_cells_to_world = T_cells_to_world;
end