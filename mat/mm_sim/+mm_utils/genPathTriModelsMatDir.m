function pathTriModelsMatDir = genPathTriModelsMatDir(sectionId,simVersion)
%GENPATHTRIMODELSMATDIR
%
% pathTriModelsMatDir = GENPATHTRIMODELSMATDIR(sectionId,simVersion)
%
% sectionId           -
% simVersion          -
%
% pathTriModelsMatDir -

pathTriModelsMatDir = sprintf('/usr0/home/atallav1/lidar_sim/mat/data/sections/section_%02d/mm_sim/version_%s', ...
    sectionId,simVersion);
end

