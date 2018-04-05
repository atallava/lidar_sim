function [primitivesCell, obbsCell] = loadClassMeshPrimitives(className, elementIds)
%LOADCLASSMESHPRIMITIVES
%
% [primitivesCell, obbsCell] = LOADCLASSMESHPRIMITIVES(className, elementIds)
%
% className      - string.
% elementIds     - nElements length vector. defaults to all elements in
% class.
%
% primitivesCell - [1,nElements] cell array.
% obbsCell       - [1,nElements] cell array.

if nargin < 2
    pattern = '([0-9]+)';
    relPathDir = mm_utils.genRelPathMeshPrimitivesDir(className);
    [~, elementIds] = getPatternMatchingFileIds(relPathDir, pattern);
end

nElements = length(elementIds);
primitivesCell = cell(1,nElements);
obbsCell = cell(1,nElements);
for i = 1:nElements
    elementId = elementIds(i);
    relPathPrimitive = ...
        mm_utils.genRelPathMeshPrimitive(className, elementId);
    can = load(relPathPrimitive,'triModels','obb');
    primitivesCell{i} = can;
    obbsCell{i} = can.obb;
end
end