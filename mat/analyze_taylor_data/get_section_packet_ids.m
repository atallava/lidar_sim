relPathSectionFrameIds = 'section_frame_ids';
load(relPathSectionFrameIds,'sectionFrameIds');

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
% point clouds extracted from sectionPacketIds(i,1) to
% sectionPacketIds(i,2)-1

%% write to txt file
relPathSectionPacketIds = 'section_packet_ids.txt';
fid = fopen(relPathSectionPacketIds,'w');
for i = 1:size(sectionPacketIds,1)
    line = sprintf('%d %d\n',sectionPacketIds(i,1),sectionPacketIds(i,2));
    fprintf(fid,line);
end
fclose(fid);

%% write to mat
relPathSectionPacketIds = 'section_packet_ids';
save(relPathSectionPacketIds,'sectionPacketIds','sectionTimes');