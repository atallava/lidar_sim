function [primitivesCell,primitiveObbsCell] = loadAllPrimitives(sectionId,primitivesVersion,primitiveClasses,primitiveClassIsPatch,classElementIds)
%LOADALLPRIMITIVES
%
% [primitivesCell,primitiveObbsCell] = LOADALLPRIMITIVES(sectionId,primitiveClasses,primitiveClassIsPatch)
%
% sectionId             - scalar.
% primitiveClasses      - cell vector of strings.
% primitiveClassIsPatch - vector.
%
% primitivesCell        - cell of cells.
% primitiveObbsCell     - cell of cells.

if nargin < 4
    classElementIds = getPrimitiveElementIds(sectionId,primitivesVersion,primitiveClasses);
end

% primitives
nPrimitiveClasses = length(primitiveClasses);
primitivesCell = cell(1,nPrimitiveClasses);
primitiveObbsCell = cell(1,nPrimitiveClasses);

for i = 1:nPrimitiveClasses
    thisClass = primitiveClasses{i};
    thisClassElementIds = classElementIds{i};
    if ~primitiveClassIsPatch(i)
        [thisClassPrimitivesCell,thisClassPrimitiveObbsCell] = loadClassPrimitives(sectionId,primitivesVersion,thisClass,thisClassElementIds);
    else
        [thisClassPrimitivesCell,thisClassPrimitiveObbsCell] = loadPatchClassPrimitives(sectionId,primitivesVersion,thisClass,thisClassElementIds);
    end
    primitivesCell{i} = thisClassPrimitivesCell;
    primitiveObbsCell{i} = thisClassPrimitiveObbsCell;
end
end