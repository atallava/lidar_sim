function pathPrimitive = genRelPathPatchPrimitiveCell(sectionId,className,elementId,cellId)
%GENPATHPATCHPRIMITIVECELL The 'cell' refers to primitive unit, not matlab
% data structure.
%
% pathPrimitive = GENPATHPATCHPRIMITIVECELL(sectionId,className,elementId)
%
% sectionId     -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('../data/sections/section_%02d/primitives/%s/%d/%d.mat', ...
    sectionId,className,elementId,cellId);
end