function [classPrimitivesCell,classPrimitiveObbsCell] = loadClassPrimitivesCell(primitiveClasses,classElementIds,primitiveClassIsPatch)

% primitives
nPrimitiveClasses = length(primitiveClasses);
classPrimitivesCell = cell(1,nPrimitiveClasses);
classPrimitiveObbsCell = cell(1,nPrimitiveClasses);

for i = 1:nPrimitiveClasses
    % this class data
    className = primitiveClasses{i};
    thisClassElementIds = classElementIds{i};
    
    % load primitives and obbs
    if ~primitiveClassIsPatch(i)
        [thisClassPrimitives,thisClassPrimitiveObbs] = loadPrimitives(thisClassElementIds);
    else
        for j = 1:nElementsThisClass
            elementId = thisClassElementIds(j);
            relPathPrimitivePatch = genRelPathPrimitivePatch(trainSectionId,className,sampledElementId);
            nObjectCells = length(objectAnnotation.T_cells_to_world);
            pattern = '([0-9]+)';
            [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
            for k = 1:nObjectCells
                sampledCellId = sampledCellIds(k);
                relPathPrimitivePatchCell = genRelPathPrimitivePatchCell(trainSectionId,className,sampledElementId,sampledCellId);
                load(relPathPrimitivePatchCell,'pts','ellipsoidModels','obb');
            end
        end
    end
    
    % add to list
    classPrimitivesCell{i} = thisClassPrimitives;
    classPrimitiveObbsCell{i} = thisClassPrimitiveObbs;
end

end

function loadPrimitives(classElementIds)
nElementsThisClass = length(classElementIds);
classPrimitives = cell(1,nElementsThisClass);
classPrimitiveObbs = cell(1,nElementsThisClass);
for i = 1:nElementsThisClass
    elementId = classElementIds(i);
    relPathPrimitive = genRelPathPrimitive(trainSectionId,className,elementId);
    can = load(relPathPrimitive,'pts','ellipsoidModels','obb');
    classPrimitives{i} = can;
    classPrimitiveObbs{i} = can.obb;
end
end

function loadPatchPrimitives()
end