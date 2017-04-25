%% rel path helpers

genRelPathDir = @(sectionId) sprintf('../data/sections/section_%02d/non_ground_segmentation', ...
    sectionId);

genRelPathPtsMat = @(sectionId,ptsId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat',sectionId,ptsId);

%% load pts
sectionId = 3;
relPathDir = genRelPathDir(sectionId);
pattern = '([0-9]+).mat';
[matchingFiles,fileIds] = getPatternMatchingFileIds(relPathDir,pattern);

nSegments = length(fileIds);
ptsCell = cell(1,nSegments);
for i = 1:length(fileIds)
    segmentId = fileIds(i);
    relPathPtsMat = genRelPathPtsMat(sectionId,segmentId);
    load(relPathPtsMat,'pts');
    ptsCell{i} = pts;
end

%% divide into sets
% make a tape
tape = calcPtsCellTape(ptsCell);

%% divide into sets
maxPtsPerSet = 50;
setCell = splitVecIntoSets(tape,maxPtsPerSet);

%%


%%

% how many classes?

% how many colors to cycle through?

% generate that many distinct colors

% choose a selector color

% create a 'tape' to help loop, based on centroid distance, say

% then the figure handler which has the callbacks allowing you to loop
% through

% play with the looping a bit

% the classification dialog

% if classified, change its color!

% first do all this without a function, just manually

% should be able to save a classification

%%
% what is the maximum points i can play with? 90k seems fine, which is 
% 50 points

hfig = figure;
hold on; axis equal;
box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
nSegments = 50;
totalPts = 0;
for i = 1:nSegments
    thisPts = ptsCell{i};
    scatter3(thisPts(:,1),thisPts(:,2),thisPts(:,3),'b.');
    totalPts = totalPts+size(thisPts,1);
end
