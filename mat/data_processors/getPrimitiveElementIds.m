function elementIdsPerClass = getPrimitiveElementIds(sectionId,primitivesVersion,primitiveClasses)
%GETPRIMITIVEELEMENTIDS
%
% elementIdsPerClass = GETPRIMITIVEELEMENTIDS(sectionId,primitiveClasses)
%
% sectionId          -
% primitiveClasses   -
%
% elementIdsPerClass -

pattern = '([0-9])+';
nClasses = length(primitiveClasses);
elementIdsPerClass = cell(1,nClasses);
for i = 1:nClasses
    className = primitiveClasses{i};
    relPathDir = genRelPathPrimitiveDir(sectionId,primitivesVersion,className);
    [~,elementIdsThisClass] = getPatternMatchingFileIds(relPathDir,pattern);
    elementIdsPerClass{i} = elementIdsThisClass;
end
end