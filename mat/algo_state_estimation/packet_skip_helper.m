% experiment design
% common variables
nPacketsUnsubsampled = (1312637-1101873)+1; % for section 4
nSubsampleSkip = 30;
nPacketsSubsampled = ceil(nPacketsUnsubsampled/nSubsampleSkip); 
nStepPerScan = 200;
packetPeriod = 5e-4; % in sec

%% given nPacketsSimulated, skipBetweenScans, skipWithinScan, calculate lastIdxProcessed
nSimSkip = 10;
nPacketsSimulated = floor(nPacketsSubsampled/nSimSkip);
skipBetweenScans = 100;
skipWithinScan = 3;

nPacketsPerScan = floor(nStepPerScan/skipWithinScan);
nScans = floor(nPacketsSimulated/nPacketsPerScan);
lastIdxProcessed = nScans*nStepPerScan + skipBetweenScans*(nScans-1);

%% given lastIdxProcessed, skipBetweenScans, skipWithinScan, calculate nPacketsSimulated
trajFracn = 1;
lastIdxProcessed = floor(nPacketsUnsubsampled*trajFracn);
skipBetweenScans = 6000;
skipWithinScan = 1;

nPacketsPerScan = floor(nStepPerScan/skipWithinScan);
nScans = (lastIdxProcessed + skipBetweenScans)/(nStepPerScan + skipBetweenScans);
nScans = floor(nScans);
nPacketsSimulated = nPacketsPerScan*nScans;

%% disp
durationBetweenScans = skipBetweenScans*packetPeriod;

fprintf('nPacketsUnsubsampled: %d\n',nPacketsUnsubsampled);
fprintf('nPacketsSubsampled: %d\n',nPacketsSubsampled);
fprintf('nStepPerScan: %d\n',nStepPerScan);
fprintf('skipWithinScan: %d\n',skipWithinScan);
fprintf('skipBetweenScans: %d\n',skipBetweenScans);
fprintf('durationBetweenScans: %.3f\n',durationBetweenScans);
fprintf('nPacketsPerScan: %d\n',nPacketsPerScan);
fprintf('nScans: %d\n',nScans);
fprintf('nPacketsSimulated: %d\n',nPacketsSimulated);
fprintf('lastIdxProcessed: %d\n',lastIdxProcessed);
fprintf('trajFracn: %.3f\n',lastIdxProcessed/nPacketsUnsubsampled);
fprintf('\n');