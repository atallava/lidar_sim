function ids = getTriModelBlockIds(relPathTriModelsDir,sectionId)
%GETTRIMODELBLOCKIDS
%
% ids = GETTRIMODELBLOCKIDS(relPathTriModelsDir,sectionId)
%
% relPathTriModelsDir -
% sectionId           -
%
% ids                 -

pattern = sprintf('section_%02d_block_([0-9]+)_ground_triangles',sectionId);
[~,ids] = getPatternMatchingFileIds(relPathTriModelsDir,pattern);
end