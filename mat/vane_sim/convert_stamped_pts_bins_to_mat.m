% rel paths to bins and mats
relPathBinsPre = '../../data/taylorJune2014/sections/world_frame/section_'; 
relPathBinsPost = 'world_frame_subsampled_timed.xyz';
genRelPathBin = @(sectionId) sprintf('%s%02d_%s',relPathBinsPre,sectionId,relPathBinsPost);

sectionIds = [3,8,12];
relPathBinCell = {};
relPathMatCell = {};
count = 0;
for sectionId = sectionIds
    count = count+1;
    relPathBinCell{count} = genRelPathBin(sectionId);
    relPathMatCell{count} = sprintf('section_%02d_driveby.mat',sectionId);
end

%% convert and write
for i = 1:length(relPathBinCell)
    relPathBin = relPathBinCell{i};
    fprintf('Converting %s...\n',relPathBin);
    [ptsTLog,pts] = timestampedPtsFromXyz(relPathBin);
    save(relPathMatCell{i},'ptsTLog','pts');
end