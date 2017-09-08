function pathPrimitive = genPathPatchPrimitiveCell(sectionId,className,elementId,cellId)
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

pathPrimitive = sprintf('/usr0/home/atallav1/lidar_sim/mat/data/sections/section_%02d/primitives/%s/%d/%d.mat', ...
    sectionId,className,elementId,cellId);
end