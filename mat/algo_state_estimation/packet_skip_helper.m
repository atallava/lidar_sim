% more experiment design variables
% common variables
nPacketsUnsubsampled = (1312637-1101873)+1; % for section 4
nSubsampleSkip = 30;
nPacketsSubsampled = ceil(nPacketsUnsubsampled/nSubsampleSkip); 
nPacketsPerScan = 200;
packetPeriod = 5e-4; % in sec

fprintf('nPacketsUnsubsampled: %d\n',nPacketsUnsubsampled);
fprintf('nPacketsSubsampled: %d\n',nPacketsSubsampled);

fprintf('\n');

%% given nPacketsSimulated, lastIdxToProcess, calculate skipDuration
nSimSkip = 10; % this was all that mm could manage
nPacketsSimulated = ceil(nPacketsSubsampled/nSimSkip);
lastIdxToProcess = nPacketsUnsubsampled;

nScans = floor(nPacketsSimulated/nPacketsPerScan);
nTotalSkippedInAgg = lastIdxToProcess - (nScans*nPacketsPerScan);
nSkipBetweenScans = nTotalSkippedInAgg/(nScans-1);
skipDuration = nSkipBetweenScans*packetPeriod;

fprintf('case 1:\n');
fprintf('lastIdxToProcess: %d\n',lastIdxToProcess);
fprintf('frac of unsubsampled: %.2f\n',lastIdxToProcess/nPacketsUnsubsampled);
fprintf('nPacketsSimulated: %d\n',nPacketsSimulated);
fprintf('nScans: %d\n',nScans);
fprintf('nSkipBetweenScans: %d\n',nSkipBetweenScans);
fprintf('skipDuration: %.2f\n',skipDuration);
fprintf('\n');

%% given skipDuration, lastIdxToProcess, calculate nPacketsSimulated
skipDuration = 1; % in s
nSkipBetweenScans = floor(skipDuration/packetPeriod);
lastIdxToProcess = nPacketsUnsubsampled;

nScans = floor(lastIdxToProcess/(nPacketsPerScan+nSkipBetweenScans));
nPacketsSimulated = nScans*nPacketsPerScan;

fprintf('case 2:\n');
fprintf('lastIdxToProcess: %d\n',lastIdxToProcess);
fprintf('frac of unsubsampled: %.2f\n',lastIdxToProcess/nPacketsUnsubsampled);
fprintf('nPacketsSimulated: %d\n',nPacketsSimulated);
fprintf('nScans: %d\n',nScans);
fprintf('nSkipBetweenScans: %d\n',nSkipBetweenScans);
fprintf('skipDuration: %.2f\n',skipDuration);
fprintf('\n');

%% given skipDuration, nPacketsSimulated, calculate lastIdxToProcess
skipDuration = 3; % in s
nSkipBetweenScans = floor(skipDuration/packetPeriod);
nSimSkip = 1; % this was all that mm could manage
nPacketsSimulated = ceil(nPacketsSubsampled/nSimSkip);

nScans = floor(nPacketsSimulated/nPacketsPerScan);
lastIdxToProcess = nScans*nPacketsPerScan + nSkipBetweenScans*(nScans-1);

fprintf('case 3:\n');
fprintf('lastIdxToProcess: %d\n',lastIdxToProcess);
fprintf('frac of unsubsampled: %.2f\n',lastIdxToProcess/nPacketsUnsubsampled);
fprintf('nPacketsSimulated: %d\n',nPacketsSimulated);
fprintf('nScans: %d\n',nScans);
fprintf('nSkipBetweenScans: %d\n',nSkipBetweenScans);
fprintf('skipDuration: %.2f\n',skipDuration);
fprintf('\n');
