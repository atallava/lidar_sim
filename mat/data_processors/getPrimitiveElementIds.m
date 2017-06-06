function elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir,primitiveClasses)
    pattern = '([0-9])+';
    nClasses = length(primitiveClasses);
    elementIdsPerClass = cell(1,nClasses);
    for i = 1:nClasses
        className = primitiveClasses{i};
        relPathDir = genRelPathClassPrimitivesDir(className);
        [~,elementIdsThisClass] = getPatternMatchingFileIds(relPathDir,pattern);
        elementIdsPerClass{i} = elementIdsThisClass;
    end
end