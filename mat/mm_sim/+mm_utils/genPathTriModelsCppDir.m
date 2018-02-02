function pathTriModelsCppDir = genPathTriModelsCppDir(sectionId,simVersion)
%GENPATHTRIMODELSCPPDIR
%
% pathTriModelsCppDir = GENPATHTRIMODELSCPPDIR(sectionId,simVersion)
%
% sectionId           -
% simVersion          -
%
% pathTriModelsCppDir -

pathTriModelsCppDir = sprintf('/usr0/home/atallav1/lidar_sim/cpp/data/sections/section_%02d/mm_sim/version_%s', ...
    sectionId,simVersion);
end

