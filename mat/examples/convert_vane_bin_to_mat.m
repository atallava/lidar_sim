% rel paths to bins and mats
relPathBinsPre = '../../data/taylorJune2014/vane/'; 
relPathBinsPost = '.asc';
genRelPathBin = @(s) [relPathBinsPre s relPathBinsPost];

terrainTypes = {'veg','ground'};
dataTypes = {'train','validation','test'};
count = 1;
relPathBinCell = {};
relPathMatCell = {};
for i = 1:length(terrainTypes)
    for j = 1:length(dataTypes)
        filename = ['rim_stretch_' terrainTypes{i} '_' dataTypes{j}];
        relPathBinCell{count} = genRelPathBin(filename);
        relPathMatCell{count} = ['rim_stretch_' terrainTypes{i} '_' dataTypes{j}];
        count = count+1;
    end
end

%% convert and write
for i = 1:length(relPathBinCell)
    relPathBin = relPathBinCell{i};
    fprintf('Converting %s...\n',relPathBin);
    pts = ptsFromXyz(relPathBin);
    save(relPathMatCell{i},'pts');
end