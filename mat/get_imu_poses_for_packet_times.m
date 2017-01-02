% packet ids
relPathPacketIds = 'section_01_subsampled_packet_ids.txt';
[packetIds,packetReadTimes] = readPacketIds(relPathPacketIds);

%% pose log
relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog','tOffset');

%% 
packetReadTimesLocal = packetReadTimes-tOffset;
poseAtPacketTimes = interp1(tLog,poseLog,packetReadTimesLocal);

%% write out
relPathOut = 'imu_poses_01_subsampled.txt';
fid = fopen(relPathOut,'w');
nPackets = length(packetIds);
for i = 1:nPackets
    line = sprintf('%d %f %f %f %f %f %f\n', ...
        packetIds(i), ...
        poseAtPacketTimes(i,1),poseAtPacketTimes(i,2),poseAtPacketTimes(i,3), ...
        poseAtPacketTimes(i,4),poseAtPacketTimes(i,5),poseAtPacketTimes(i,6));
    fprintf(fid,line);
end


