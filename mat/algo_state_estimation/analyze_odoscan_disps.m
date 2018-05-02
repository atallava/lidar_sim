% disps output by odoscan

%% data params
sectionId = 4;
scansVersion = '300118';
dataSource = 'real';
sourceVersion = '';
odoscanParamsIdx = 1;

%% load
% relPathScanPoses = genRelPathScanPoses(sectionId,scansVersion,dataSource,sourceVersion);
relPathScanPoses = 'disps_vel_integration.txt';
[tReal, TCellReal] = loadScanPoses(relPathScanPoses);
nPoses = length(tReal);

relPathDisps = genRelPathDisps(sectionId,scansVersion,dataSource,sourceVersion,odoscanParamsIdx);
[tDisps, TCellDisp] = loadDisps(relPathDisps);
nDisps = length(tDisps);

%% todo: correct hack
% removing first pose from real
tReal(1) = [];
TCellReal(1) = [];
nPoses = nPoses-1;

%% timestamp integrity checks 
condn = all(diff(tDisps) > 0);
msg = 'disp timestamps not monotonically increasing.';
assert(condn,msg);

condn = (nDisps == (nPoses-1));
msg = 'expected nDisps = nPoses-1.';
assert(condn,msg);

flag = abs(tReal(2:end) - tDisps);
flag = flag < 1e-6;
condn = all(flag);
msg = 'timestamps do not agree.';
assert(condn,msg);

%% real, estimated posns
posnsReal = zeros(nPoses,3);
for i = 1:nPoses
    TReal = TCellReal{i};
    posnsReal(i,:) = TReal(1:3,4);
end

TCellEst = cell(1,nDisps+1);
posnsEst = zeros(nDisps+1,3);
TCellEst{1} = TCellReal{1};
posnsEst(1,:) = TCellEst{1}(1:3,4);
for i = 2:(nDisps+1)
    TCellEst{i} = TCellEst{i-1}*TCellDisp{i-1};
    TEst = TCellEst{i};
    posnsEst(i,:) = TEst(1:3,4);
end

%% viz
hfig = figure();
lineWidth = 1.5;
plot(posnsReal(:,1),posnsReal(:,2),'bo-', ...
    'linewidth',lineWidth);
hold on; axis equal;
plot(posnsEst(:,1),posnsEst(:,2),'rx-', ...
    'linewidth',lineWidth);
xlabel('x (m)'); ylabel('y (m)');
legend({'real','est'});

%% errors
posnErrVec = zeros(1,nPoses);
TErrVec = zeros(1,nPoses);
for i = 1:nPoses
    TReal = TCellReal{i};
    TEst = TCellEst{i};
    posnsReal(i,:) = TReal(1:3,4);
    posnsEst(i,:) = TEst(1:3,4);
    
    TErrVec(i) = norm(TReal-TEst,'fro')^2; % todo: what metric goes here?
    posnErrVec(i) = norm(posnsReal(i,:)-posnsEst(i,:))^2;
end

fprintf('mean posn err: %.2f\n',mean(posnErrVec));
fprintf('mean T err: %.2f\n',mean(TErrVec));

