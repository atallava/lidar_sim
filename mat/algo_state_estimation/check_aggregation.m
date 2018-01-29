%% helpers
genRelPathPackets = @(sectionId,version,dataSource) ...
    sprintf('../../cpp/data/algo_state_estimation/sections/section_%02d/version_%s/%s/packets_to_process.txt', ...
    sectionId,version,dataSource);

%% data signature
sectionId = 4;
processVersion = '260118';
dataSource = 'real';

%% load some of the packets to process
relPathPackets = genRelPathPackets(sectionId,processVersion,dataSource);
nPacketsToRead = 1000;
packets = loadSection(relPathPackets,nPacketsToRead);

%% do we have the right packets per scan, skip etc
plot(packets.packetIds,'.');

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId,processVersion,dataSource);
[timestamps,TCell] = loadScanPoses(relPathScanPoses);
nScans = length(TCell);
xyPosns = zeros(nScans,2);
for i = 1:nScans
    T = TCell{i};
    xyPosns(i,:) = T(1:2,4);
end

% drift between scan poses
delXyVec = zeros(1,nScans-1);
for i = 1:length(delXyVec)
    delXyVec(i) = norm(xyPosns(i+1,:)-xyPosns(i,:));
end

%% viz posns
plot(xyPosns(:,1),xyPosns(:,2),'+');
axis equal;
xlabel('x (m)'); ylabel('y (m)');
