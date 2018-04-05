function pathPrimitive = genRelPathPatchPrimitive(sectionId,primitivesVersion,className,elementId)
%GENPATHPATCHPRIMITIVE
%
% pathPrimitive = GENPATHPATCHPRIMITIVE(sectionId,primitivesVersion,className,elementId)
%
% sectionId     -
% primitivesVersion -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('../data/sections/section_%02d/primitives/version_%s/%s/%d', ...
    sectionId,primitivesVersion,className,elementId);
end