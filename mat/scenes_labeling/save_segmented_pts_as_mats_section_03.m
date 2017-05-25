genRelPathDir = @(sectionId) sprintf('../../cpp/data/sections/section_%02d/non_ground_segmentation', ...
    sectionId);

genRelPathPtsMat = @(sectionId,ptsId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat',sectionId,ptsId);

%%
sectionId = 3;
relPathDir = genRelPathDir(sectionId);
pattern = '([0-9]+).asc';
[matchingFiles,fileIds] = getPatternMatchingFileIds(relPathDir,pattern);

for i = 1:length(matchingFiles)
    fprintf('pts %d...\n',fileIds(i));
    relPathPts = sprintf('%s/%s',relPathDir,matchingFiles{i});
    pts = loadPts(relPathPts);
    relPathPtsMat = genRelPathPtsMat(sectionId,fileIds(i));
    save(relPathPtsMat,'relPathPts','pts');
end