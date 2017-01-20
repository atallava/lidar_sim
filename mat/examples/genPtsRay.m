function pts = genPtsRay(origin,dirn,segmentLength)
   if nargin < 3
       segmentLength = 10;
   end
   stepsizeVec = [0:0.1:segmentLength]';
   nPts = length(stepsizeVec);
   origin = flipVecToRow(origin);
   dirn = flipVecToRow(dirn);
   dirnMat = repmat(dirn,nPts,1);
   stepMat = bsxfun(@times,dirnMat,stepsizeVec);
   pts = bsxfun(@plus,stepMat,origin);
end