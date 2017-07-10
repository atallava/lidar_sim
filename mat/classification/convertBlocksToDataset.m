function dataset = convertBlocksToDataset(blockPts,blockLabels,blockIds)
blockPts = blockPts(blockIds);
blockLabels = blockLabels(blockIds);
dataset.pts = cell2mat(flipVecToColumn(blockPts));
dataset.labels = cell2mat(flipVecToColumn(blockLabels));
end