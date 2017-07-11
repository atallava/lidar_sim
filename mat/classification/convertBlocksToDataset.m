function dataset = convertBlocksToDataset(blockPts,blockLabels,blockIds)
%CONVERTBLOCKSTODATASET
%
% dataset = CONVERTBLOCKSTODATASET(blockPts,blockLabels,blockIds)
%
% blockPts    -
% blockLabels -
% blockIds    -
%
% dataset     -

blockPts = blockPts(blockIds);
blockLabels = blockLabels(blockIds);
dataset.pts = cell2mat(flipVecToColumn(blockPts));
dataset.labels = cell2mat(flipVecToColumn(blockLabels));
end