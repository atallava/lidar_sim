% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

genRelPathClassPrimitivesDir = @(sectionId,className) ...
    sprintf('../data/sections/section_%02d/primitives/%s',sectionId,className);

trainSectionId = 3;
genRelPathClassPrimitivesDir2 = @(className) ...
    genRelPathClassPrimitivesDir(trainSectionId,className);
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir2,primitiveClasses);

for i = 1:length(primitiveClasses)
    fprintf('class: %s, n elements: %d\n',primitiveClasses{i},length(elementIdsPerClass{i}));
end
