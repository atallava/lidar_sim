% i will aggregate packets from the unsubsampled. this makes skip necessary

nPacketsUnsubsampled = (1312637-1101873)+1;
nSubsampleSkip = 30;
nPacketsSubsampled = ceil(nPacketsUnsubsampled/nSubsampleSkip); 
nPacketsPerScan = 200;
packetPeriod = 5e-4; % in sec

fprintf('nPacketsUnsubsampled: %d\n',nPacketsUnsubsampled);
fprintf('nPacketsSubsampled: %d\n',nPacketsSubsampled);

fprintf('\n');

%% given nPacketsSimulated, nPacketsToAgg, calculate skipDuration
nSimSkip = 10;
nPacketsSimulated = ceil(nPacketsSubsampled/nSimSkip);
nPacketsToAgg = nPacketsUnsubsampled;

nScans = floor(nPacketsToAgg/nPacketsPerScan);
nTotalSkippedInAgg = nPacketsToAgg - (nScans*nPacketsPerScan);
nSkipBetweenScans = nTotalSkippedInAgg/(nScans-1);
skipDuration = nSkipBetweenScans*packetPeriod;

fprintf('case 1:');
fprintf('nPacketsToAgg: %.2f\n',nPacketsToAgg);
fprintf('nPacketsSimulated: %d\n',nPacketsSimulated);
fprintf('nScans: %d\n',nScans);
fprintf('nSkipBetweenScans: %d\n',nSkipBetweenScans);
fprintf('skipDuration: %.2f\n',skipDuration);

%% given skipDuration, nPacketsToAgg, calculate nPacketsSimulated
skipDuration = 1; % in s
nSkipBetweenScans = floor(skipDuration/packetPeriod);
nPacketsToAgg = nPacketsUnsubsampled;

nScans = floor(nPacketsToAgg/(nPacketsPerScan+nSkipBetweenScans));
nPacketsSimulated = nScans*nPacketsPerScan;

fprintf('case 1:');
fprintf('nPacketsToAgg: %.2f\n',nPacketsToAgg);
fprintf('nPacketsSimulated: %d\n',nPacketsSimulated);
fprintf('nScans: %d\n',nScans);
fprintf('nSkipBetweenScans: %d\n',nSkipBetweenScans);
fprintf('skipDuration: %.2f\n',skipDuration);

%% given skipDuration, nPacketsSimulated, calculate nPacketsToAgg
skipDuration = 1; % in s
nSkipBetweenScans = floor(skipDuration/packetPeriod);
nPacketsSimulated = ceil(nPacketsSubsampled/nSimSkip);

nScans = floor(nPacketsSimulated/nPacketsPerScan);
nPacketsToAgg = nScans*nPacketsPerScan + nSkipBetweenScans*(nScans-1);

fprintf('case 1:');
fprintf('nPacketsToAgg: %.2f\n',nPacketsToAgg);
fprintf('nPacketsSimulated: %d\n',nPacketsSimulated);
fprintf('nScans: %d\n',nScans);
fprintf('nSkipBetweenScans: %d\n',nSkipBetweenScans);
fprintf('skipDuration: %.2f\n',skipDuration);
