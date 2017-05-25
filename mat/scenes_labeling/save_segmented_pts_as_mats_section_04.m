genRelPathDir = @(sectionId,partId) sprintf('../../cpp/data/sections/section_%02d/non_ground_segmentation/part_%d', ...
    sectionId,partId);

genRelPathPtsMat = @(sectionId,ptsId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat',sectionId,ptsId);

genRelPathSegmentToPart = @(sectionId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/segment_to_part_info.mat',sectionId);

%%
sectionId = 4;
partIds = 1:3;
nParts = length(partIds); % segmentation was done in n parts
segmentCount = 1;
segmentToPartCell = {}; % keeps track of which segment came from which part

for i = 1:nParts
    % loop over parts
    partId = partIds(i);
    fprintf('part %d...\n',partId);
    relPathDir = genRelPathDir(sectionId,partId);
    
    % segments in this part
    pattern = '([0-9]+).asc';
    [matchingFiles,fileIds] = getPatternMatchingFileIds(relPathDir,pattern);
    
    for j = 1:length(matchingFiles)
        fprintf('pts %d...\n',fileIds(j));
        relPathPts = sprintf('%s/%s',relPathDir,matchingFiles{j});
        pts = loadPts(relPathPts);
        relPathPtsMat = genRelPathPtsMat(sectionId,segmentCount);
        segmentToPartCell{segmentCount} = [partId fileIds(j)];
        save(relPathPtsMat,'relPathPts','pts');
        segmentCount = segmentCount+1;
    end    
end

%%
relPathSegmentToPart = genRelPathSegmentToPart(sectionId);
save(relPathSegmentToPart,'segmentToPartCell');
