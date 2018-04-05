function [primitivesCell, obbsCell, elementMembership] = loadPatchClassMeshPrimitives(className, elementIds)
%LOADPATCHCLASSMESHPRIMITIVES
%
% [primitivesCell, obbsCell] = LOADPATCHCLASSMESHPRIMITIVES(className, elementIds)
%
% className      - string.
% elementIds     - nElements length vector. defaults to all elements in
% class.
%
% primitivesCell - \sum_i nCells_i length cell vector.
% obbsCell       - \sum_i nCells_i length cell vector.
% elementMembership - \sum_i nCells_i length cell vector.

if nargin < 2
    pattern = '([0-9]+)';
    relPathDir = mm_utils.genRelPathMeshPrimitivesDir(className);
    [~, elementIds] = getPatternMatchingFileIds(relPathDir, pattern);
end

nElements = length(elementIds);
% leaving these empty because haven't computed how many cells per element
primitivesCell = {};
obbsCell = {};
elementMembership = [];
for i = 1:nElements
    elementId = elementIds(i);

    % cell ids for this element
    relPathPrimitive = mm_utils.genRelPathMeshPrimitive(className,elementId);
    relPathPrimitive = strrep(relPathPrimitive, '.mat', '');
    pattern = '([0-9])+';
    [~,cellIds] = getPatternMatchingFileIds(relPathPrimitive,pattern);
    nCells = length(cellIds);
    
    for j = 1:nCells
        cellId = cellIds(j);
        
        % load cell
        relPathCell = ...
            mm_utils.genRelPathMeshPrimitivePatchCell(className, elementId, cellId);
        can = load(relPathCell,'triModels','obb');
        
        % add to big list
        primitivesCell{end+1} = can;
        obbsCell{end+1} = can.obb;
    end
    
    elementMembership = [elementMembership repmat(elementId,1,nCells)];
end
end