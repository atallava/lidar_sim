function relPathDir = genRelPathSceneTriModelsMatDir(sectionId, simVersion)
%GENRELPATHSCENETRIMODELSMATDIR
%
% relPathDir = GENRELPATHSCENETRIMODELSMATDIR(sectionId, simVersion)
%
% sectionId  -
% simVersion -
%
% relPathDir -

relPathDir = sprintf('../data/sections/section_%02d/mm_sim/version_%s', ...
    sectionId, simVersion);
end