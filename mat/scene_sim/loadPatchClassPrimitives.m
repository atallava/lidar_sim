function [primitivesCell,obbsCell,elementMembership] = loadPatchClassPrimitives(sectionId,className,elementIds)
%LOADPATCHCLASSPRIMITIVES
%
% [primitivesCell,obbsCell] = LOADPATCHCLASSPRIMITIVES(sectionId,className,elementIds)
%
% sectionId      - scalar.
% className      - string.
% elementIds     - nElements length vector.
%
% primitivesCell - nElements*nCells cell vector.
% obbsCell       - nElements*nCells cell vector.

if nargin < 3
    pattern = '([0-9])+';
    relPathDir = genRelPathPrimitiveDir(sectionId,className);
    [~,elementIds] = getPatternMatchingFileIds(relPathDir,pattern);
end

nElements = length(elementIds);
% leaving these empty because haven't computer how many cells per element
primitivesCell = {};
obbsCell = {};
elementMembership = [];
for i = 1:nElements
    elementId = elementIds(i);
    
    % cell ids for this element
    pathPatchPrimitive = genRelPathPatchPrimitive(sectionId,className,elementId);
    pattern = '([0-9])+';
    [~,cellIds] = getPatternMatchingFileIds(pathPatchPrimitive,pattern);
    nCells = length(cellIds);
    
    for j = 1:nCells
        cellId = cellIds(j);
        % load cell
        pathPatchPrimitiveCell = genRelPathPatchPrimitiveCell(sectionId,className,elementId,cellId);
        can = load(pathPatchPrimitiveCell,'pts','ellipsoidModels','obb');
        
        % add to big list
        primitivesCell{end+1} = can;
        obbsCell{end+1} = can.obb;
        elementMembership = [elementMembership repmat(elementId,1,nCells)];
    end
end
end
