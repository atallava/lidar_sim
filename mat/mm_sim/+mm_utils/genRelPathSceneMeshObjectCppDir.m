function relPathDir = genRelPathSceneMeshObjectCppDir(sectionId, simVersion)
%GENRELPATHSCENEMESHOBJECTCPPDIR
%
% relPathDir = GENRELPATHSCENEMESHOBJECTCPPDIR(sectionId, simVersion)
%
% sectionId  -
% simVersion -
%
% relPathDir -

relPathDir = ...
    sprintf('../../cpp/data/sections/section_%02d/mm_sim/version_%s', ...
    sectionId,simVersion);
end