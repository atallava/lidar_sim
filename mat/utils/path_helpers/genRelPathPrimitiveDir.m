function pathPrimitiveDir = genRelPathPrimitiveDir(sectionId,className)
%GENPATHPRIMITIVEDIR
%
% pathPrimitiveDir = GENPATHPRIMITIVEDIR(sectionId,className)
%
% sectionId        -
% className        -
%
% pathPrimitiveDir -

pathPrimitiveDir = sprintf('../data/sections/section_%02d/primitives/%s', ...
    sectionId,className);
end