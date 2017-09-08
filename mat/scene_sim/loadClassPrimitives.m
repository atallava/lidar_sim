function [primitivesCell,obbsCell] = loadClassPrimitives(sectionId,className,elementIds)
%LOADCLASSPRIMITIVES
%
% [primitivesCell,obbsCell] = LOADCLASSPRIMITIVES(sectionId,className,elementIds)
%
% sectionId      -
% className      -
% elementIds     -
%
% primitivesCell -
% obbsCell       -

if nargin < 3
    pattern = '([0-9])+';
    relPathDir = genPathPrimitiveDir(sectionId,className);
    [~,elementIds] = getPatternMatchingFileIds(relPathDir,pattern);
end

nElements = length(elementIds);
primitivesCell = cell(1,nElements);
obbsCell = cell(1,nElements);
for i = 1:nElements
    elementId = elementIds(i);
    relPathPrimitive = genPathPrimitive(sectionId,className,elementId);
    can = load(relPathPrimitive,'pts','ellipsoidModels','obb');
    primitivesCell{i} = can;
    obbsCell{i} = can.obb;
end
end