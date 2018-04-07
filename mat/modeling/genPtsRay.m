function pts = genPtsRay(origin,dirn,segmentLength)
%GENPTSRAY
%
% pts = GENPTSRAY(origin,dirn,segmentLength)
%
% origin        - length 3 vector.
% dirn          - length 3 vector.
% segmentLength - scalar. defaults to 10.
%
% pts           - [nPts,3] array.

if nargin < 3
    segmentLength = 10;
end
nSteps = 10;
stepsizeVec = linspace(0, segmentLength, nSteps)';
nPts = length(stepsizeVec);
origin = flipVecToRow(origin);
dirn = flipVecToRow(dirn);
dirnMat = repmat(dirn,nPts,1);
stepMat = bsxfun(@times,dirnMat,stepsizeVec);
pts = bsxfun(@plus,stepMat,origin);
end