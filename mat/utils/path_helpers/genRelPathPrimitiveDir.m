function pathPrimitiveDir = genRelPathPrimitiveDir(sectionId,primitivesVersion,className)
%GENRELPATHPRIMITIVEDIR
%
% pathPrimitiveDir = GENRELPATHPRIMITIVEDIR(sectionId,primitivesVersion,className)
%
% sectionId         -
% primitivesVersion -
% className         -
%
% pathPrimitiveDir  -

pathPrimitiveDir = sprintf('../data/sections/section_%02d/primitives/version_%s/%s', ...
    sectionId,primitivesVersion,className);
end