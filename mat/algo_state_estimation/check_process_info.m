% i found an error between n scans reported by process info,
% and the actual number of scans generated

nPackets = 210765; % number of packets in section 4 unsubsampled
nStepPerScan = 200;
nSkipWithinScan = 1;
nSkipBetweenScans = 500;

%% mimic of code in create_real_packets...
nPacketsPerScan = nStepPerScan/nSkipWithinScan;
condn = mod(nPacketsPerScan,10) == 0;
msg = 'n packets per scan must be integer';
assert(condn,msg);

scanStartIdx = 0;
nScans = 0;
scanIds = {};
while (scanStartIdx < (nPackets-1))
    scanEndIdx = scanStartIdx + (nStepPerScan-1);
    if (scanEndIdx > (nPackets-1))
        break;
    end
    scanIds{end+1} = [scanStartIdx:nSkipWithinScan:scanEndIdx];
    nScans = nScans+1;
    scanStartIdx = nSkipBetweenScans + (scanEndIdx + 1);
end

fprintf('nPacketsPerScan: %d\n',nPacketsPerScan);
fprintf('nScans: %d\n',nScans);

%% mimic of code in PacketsToScanAggregator
% aggregator loads the correct number of packets to process
% and n packets per process

nPacketsToProcess = nScans*nPacketsPerScan;
nScansAggregated = 0;

scanStartIdx = 0;
% found error: way ids used here don't match previous
while (scanStartIdx < nPacketsToProcess)
    scanEndIdx = scanStartIdx + nPacketsPerScan;
    if (scanEndIdx >= nPacketsToProcess)
        break;
    end
    nScansAggregated = nScansAggregated+1;
    scanStartIdx = scanEndIdx + 1;
end

fprintf('nScansAggregated: %d\n',nScansAggregated);