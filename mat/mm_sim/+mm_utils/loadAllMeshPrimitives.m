function [primitivesCell, primitiveObbsCell] = loadAllMeshPrimitives(primitiveClasses,primitiveClassIsPatch,elementIdsPerClass)
%LOADALLMESHPRIMITIVES
%
% primitivesPerClass = LOADALLMESHPRIMITIVES(primitiveClasses,primitiveClassIsPatch,elementIdsPerClass)
%
% primitiveClasses      - length nClasses cell array. 
% primitiveClassIsPatch - length nClasses vector. 
% elementIdsPerClass    - length nClasses cell array. 
%
% primitivesCell        - cell of cells.
% primitiveObbsCell     - cell of cells.

nClasses = length(primitiveClasses);
primitivesCell = cell(1,nClasses);
primitiveObbsCell = cell(1,nPrimitiveClasses);

for i = 1:nClasses
    thisClass = primitiveClasses{i};
    elementIds = elementIdsPerClass{i};
    
    if ~primitiveClassIsPatch(i)
        [thisClassPrimitives, thisObbsCell] = mm_utils.loadClassMeshPrimitives(thisClass, elementIds);
    else
        [thisClassPrimitives, thisObbsCell] = mm_utils.loadPatchClassMeshPrimitives(thisClass, elementIds);
    end
    primitivesCell{i} = thisClassPrimitives;
    primitiveObbsCell{i} = thisObbsCell;
end
end
