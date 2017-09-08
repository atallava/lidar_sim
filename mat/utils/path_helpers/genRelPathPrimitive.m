function pathPrimitive = genRelPathPrimitive(sectionId,className,elementId)
%GENPATHPRIMITIVE
%
% pathPrimitive = GENPATHPRIMITIVE(sectionId,className,elementId)
%
% sectionId     -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('../data/sections/section_%02d/primitives/%s/%d.mat', ...
    sectionId,className,elementId);
end