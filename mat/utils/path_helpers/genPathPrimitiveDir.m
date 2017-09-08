function pathPrimitiveDir = genPathPrimitiveDir(sectionId,className)
%GENPATHPRIMITIVEDIR
%
% pathPrimitiveDir = GENPATHPRIMITIVEDIR(sectionId,className)
%
% sectionId        -
% className        -
%
% pathPrimitiveDir -

pathPrimitiveDir = sprintf('/usr0/home/atallav1/lidar_sim/mat/data/sections/section_%02d/primitives/%s', ...
    sectionId,className);
end