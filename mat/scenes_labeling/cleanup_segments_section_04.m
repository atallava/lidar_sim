%% rel path helpers
genRelPathLabelingSetsInfo = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_sets_info.mat',sectionId);

%% load
sectionId = 4;
relPathLabelingSetsInfo = genRelPathLabelingSetsInfo(sectionId);
load(relPathLabelingSetsInfo,'ptsCell','tape','setCell');

%% find copies
% get first point of each segment
nSegments = length(ptsCell);
segmentRefPts = zeros(nSegments,3);
for i = 1:length(ptsCell)
    pts = ptsCell{i};
    segmentRefPts(i,:) = pts(1,:);
end

uniquePts = unique(segmentRefPts,'rows');
repeatSets = {};
for i = 1:size(uniquePts,1)
    set = [];
    for j = 1:size(segmentRefPts,1)
        if isequal(uniquePts(i,:),segmentRefPts(j,:))
            set = [set j];
        end
    end
    if length(set) > 1
        repeatSets{end+1} = set;
    end
end