function [primitivesCell,obbsCell] = loadClassPrimitives(sectionId,primitivesVersion,className,elementIds)
%LOADCLASSPRIMITIVES
%
% [primitivesCell,obbsCell] = LOADCLASSPRIMITIVES(sectionId,className,elementIds)
%
% sectionId      - scalar.
% className      - string.
% elementIds     - nElements length vector.
%
% primitivesCell - nElements length cell array.
% obbsCell       - nElements length cell array.

if nargin < 3
    pattern = '([0-9])+';
    relPathDir = genRelPathPrimitiveDir(sectionId,primitivesVersion,className);
    [~,elementIds] = getPatternMatchingFileIds(relPathDir,pattern);
end

nElements = length(elementIds);
primitivesCell = cell(1,nElements);
obbsCell = cell(1,nElements);
for i = 1:nElements
    elementId = elementIds(i);
    relPathPrimitive = genRelPathPrimitive(sectionId,primitivesVersion,className,elementId);
    can = load(relPathPrimitive,'pts','ellipsoidModels','obb');
    primitivesCell{i} = can;
    obbsCell{i} = can.obb;
end
end