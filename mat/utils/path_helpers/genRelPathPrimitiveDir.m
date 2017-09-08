function pathPrimitiveDir = genRelPathPrimitiveDir(sectionId,primitivesVersion,className)
%GENPATHPRIMITIVEDIR
%
% pathPrimitiveDir = GENPATHPRIMITIVEDIR(sectionId,className)
%
% sectionId        -
% className        -
%
% pathPrimitiveDir -

pathPrimitiveDir = sprintf('../data/sections/section_%02d/primitives/version_%s/%s', ...
    sectionId,primitivesVersion,className);
end