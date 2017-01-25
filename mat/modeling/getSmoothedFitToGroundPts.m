function ptsFit = getSmoothedFitToGroundPts(pts,triParams)
%GETSMOOTHEDFITTOGROUNDPTS 
% 
% ptsFit = GETSMOOTHEDFITTOGROUNDPTS(pts)
% 
% pts    - [nPts,3] array.
% triParams - struct.
% 
% ptsFit - [nPtsFit,3] array.

[xNodeVec,yNodeVec] = genNodeVecsForSmoothedFit(pts,triParams.ptsFitParams.padding,triParams.ptsFitParams.nodeResn);

smoothness = triParams.ptsFitParams.smoothness;
% what is the effect of smoothness?
% default is 5e-3. visible difference to 1e-3
% 5e-2 already smooths too much
% 5e-4 seems to capture some amount of structure?
zFitMat = RegularizeData3D(pts(:,1),pts(:,2),pts(:,3),xNodeVec,yNodeVec,'interp','bicubic','smoothness',smoothness);

[xFitUnrolled,yFitUnrolled,zFitUnrolled] = unrollFitPts(xNodeVec,yNodeVec,zFitMat);
ptsFit = [xFitUnrolled, yFitUnrolled, zFitUnrolled];

ptsFit = restrictFitToObsProjn(pts,ptsFit,triParams.ptsFitParams.maxDistToProjn);
end