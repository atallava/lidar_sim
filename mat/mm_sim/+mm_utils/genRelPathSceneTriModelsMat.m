function relPathModels = genRelPathSceneTriModelsMat(sectionId, simVersion)
%GENRELPATHSCENETRIMODELSMAT
%
% relPathModels = GENRELPATHSCENETRIMODELSMAT(sectionId, simVersion)
%
% sectionId     - integer.
% simVersion    - string.
%
% relPathModels - string.

relPathDir = mm_utils.genRelPathSceneTriModelsMatDir(sectionId, simVersion);
relPathModels = [relPathDir '/scene_tri_models.mat'];
end