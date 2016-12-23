%% section frame ids
sectionFrameIds = [4375 5200; ... % loop A
    5200 6250; ... % loop A
    8180 8870; ... % rim stretch
    9300 10900; ... % loop B
    10900 12410; ... % loop B
    12900 13800; ... % loop C
    13800 15000; ... % loop C
    15840 16380; ... % rim stretch
    17900 20090; ... % loop D
    20090 21600; ... % loop D
    24880 28340; ... % loop C'
    30670 31330; ... % rim stretch
    32120 33200; ... % loop A
    33200 33900]; % loop A
    
%% section times
sectionTimes = arrayfun(@getTimeFromFrameId,sectionFrameIds);

% pad times
padding = [-10 10]; 
sectionTimes = bsxfun(@plus,sectionTimes,padding);

% don't want end of section to be after start of next section
nSections = size(sectionTimes,1);
for i = 1:(nSections-1)
    if sectionTimes(i,2) > sectionTimes(i+1,1)
        sectionTimes(i,2) = (sectionTimes(i,2)+sectionTimes(i+1,1))*0.5;
        sectionTimes(i+1,1) = sectionTimes(i,2)+1;
    end
end

%% packet ids
load('velodyne_packet_times','packetTimes');
sectionPacketIds = arrayfun(@(t) indexOfNearestTime(t,packetTimes),sectionTimes);

%% write to txt file
relPathPacketIds = 'velo_packet_ids';
fid = fopen(relPathPacketIds,'w');
for i = 1:size(sectionPacketIds,1)
    line = sprintf('%d %d\n
end
