function ids = getEllipsoidModelBlockIds(relPathEllipsoidModelsDir,sectionId)
%GETELLIPSOIDMODELBLOCKIDS
%
% ids = GETELLIPSOIDMODELBLOCKIDS(relPathEllipsoidModelsDir,sectionId)
%
% relPathEllipsoidModelsDir -
% sectionId                 -
%
% ids                       -

pattern = sprintf('section_%02d_block_([0-9]+)_non_ground_ellipsoids',sectionId);
[~,ids] = getPatternMatchingFileIds(relPathEllipsoidModelsDir,pattern);
end