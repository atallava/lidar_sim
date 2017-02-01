relPathPts = '../data/rim_stretch_segment_train';
load(relPathPts,'pts');

%% subsample
skip = 14;
pts = pts(1:skip:end,:);

%% calc spherical variation
nPts = size(pts,1);
D = pdist2(pts,pts);
maxDistToNbr = 5;
minNbrs = 15;
sphVarnVec = zeros(nPts,1);
sphVarnDefault = 0.01;
nPtsInsufficientNbrs = 0;

clockLocal = tic();
for i = 1:nPts
    thisPt = pts(i,:);
    thisPtDistances = D(i,:);
    
    nbrFlag = thisPtDistances < maxDistToNbr;
    thisNbrPts = pts(nbrFlag,:);
        
    if sum(nbrFlag) < minNbrs
        sphVarnVec(i) = sphVarnDefault;
        nPtsInsufficientNbrs = nPtsInsufficientNbrs+1;
    else
        sphVarnVec(i) = calcSphericalVariation(thisNbrPts);
    end
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);
fprintf('fracn with insufficient nbrs: %.2f\n',nPtsInsufficientNbrs/nPts*100);

%% segment
sphVarnThresh = 0.1;
groundFlag = sphVarnVec < sphVarnThresh;
ptsGround = pts(groundFlag,:);
ptsNonGround = pts(~groundFlag,:);
fprintf('fracn ground: %.2f\n',sum(groundFlag)/nPts*100);

%% viz
classLabels = {'non-ground','ground'};
hfig = vizSegmentation(pts,groundFlag,classLabels);

% quick set of parameters which work:
% maxDistForNbr = 5
% minNbrs = 15
% sphVarnThresh = 0.1

% add-in filtering over s values, which has its own set of n parameters