% i forgot the purpose of this script

%% rel path helpers
genRelPathPrimitivePts = @(sectionId,className,elementId) ...
    sprintf('../../cpp/data/sections/section_%02d/primitives/pts/%s_%d.asc',sectionId,className,elementId);

genRelPathPrimitiveEllipsoids = @(sectionId,className,elementId) ...
    sprintf('../../cpp/data/sections/section_%02d/primitives/ellipsoids/%s_%d.txt',sectionId,className,elementId);

%%
sectionId = 3;
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

relPathPrimitiveElementsDir = '../../cpp/data/sections/section_03/primitives/pts';

nClasses = length(primitiveClasses);
sectionId = 3;
elementIdsCell = cell(1,nClasses);
elementPtsCell = {};
for i = 1:nClasses
    primitiveClass = primitiveClasses{i};
    fprintf('loading pts for class %s...\n',primitiveClass);
    pattern = sprintf('%s_([0-9]+).asc',primitiveClass);
    [~,elementIds] = getPatternMatchingFileIds(relPathPrimitiveElementsDir,pattern);
    
    nElements = length(elementIds);
    elementPts = cell(1,nElements);
    for j = 1:nElements
        elementId = elementIds(j);
        relPathPrimitivePts = genRelPathPrimitivePts(sectionId,primitiveClass,elementId);
        elementPts{j} = loadPts(relPathPrimitivePts);
    end
    elementIdsCell{i} = elementIds;
    elementPtsCell{i} = elementPts;
end

%%
relPathOut = '../data/section_03_primitives_data';
save(relPathOut,'sectionId','primitiveClasses','elementIdsCell','elementPtsCell');