function classId = getClassIdFromClassName(className,primitiveClasses)
    classId = find(strcmp(primitiveClasses,className));
end