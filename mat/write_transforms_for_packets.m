% load data
relPathSectionPacketIds = 'section_packet_ids';
load(relPathSectionPacketIds,'sectionPacketIds','sectionTimes');

relPathPacketTimes = 'velodyne_packet_times';
load(relPathPacketTimes,'packetTimes');

relPathPoses25 = 'pose25_log';
load(relPathPoses25,'pose25Log','tLog');
poseTLog = tLog;

%%
relPathTransfsPre = '../data/taylorJune2014/sections/imu_transfs';
nSections = size(sectionPacketIds,1);
refHeight = 0;

clockLocal = tic();
for i = 1:nSections
    relPathTransfs = [relPathTransfsPre '/'...
        'imu_transfs_' sprintf('%02d',i) '.txt'];
    fid = fopen(relPathTransfs,'w');
    fprintf('Writing to: %s\n',relPathTransfs);
    
    startId = sectionPacketIds(i,1);
    endId = sectionPacketIds(i,2); % actually this -1
    
    for j = startId:endId
        t = packetTimes(j);
        pose25t = interp1(poseTLog,pose25Log,t);
        T = pose25ToTransform(pose25t,refHeight);
    
        line = [int2str(j) ' ' num2str(T(:)')];
        fprintf(fid,line);
        fprintf(fid,'\n');
    end
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);