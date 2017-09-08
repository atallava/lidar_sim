function pathPrimitive = genRelPathPatchPrimitive(sectionId,className,elementId)
%GENPATHPATCHPRIMITIVE
%
% pathPrimitive = GENPATHPATCHPRIMITIVE(sectionId,className,elementId)
%
% sectionId     -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('../data/sections/section_%02d/primitives/%s/%d', ...
    sectionId,className,elementId);
end