function viewDirns = calcViewDirns(cameraPositions,cameraTargets)
viewDirns = cameraTargets-cameraPositions;
rowNorms = sum(viewDirns.^2,2);
viewDirns = bsxfun(@rdivide,viewDirns,rowNorms);
end