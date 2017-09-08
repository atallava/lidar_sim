function pathPrimitive = genRelPathPrimitive(sectionId,primitivesVersion,className,elementId)
%GENPATHPRIMITIVE
%
% pathPrimitive = GENPATHPRIMITIVE(sectionId,className,elementId)
%
% sectionId     -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('../data/sections/section_%02d/primitives/version_%s/%s/%d.mat', ...
    sectionId,primitivesVersion,className,elementId);
end