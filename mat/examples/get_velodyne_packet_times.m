initLocal;
pathPacketTimestamp = [pathTaylorData '/Velodyne/Velodyne_Packet_Timestamp.txt'];

%%
fid = fopen(pathPacketTimestamp);
line = fgetl(fid); % fields description line
line = fgetl(fid);
count = 1;
packetTimes = [];
while ischar(line)
    c = strsplit(line);
    packetTimes(count) = str2num(c{2});
    line = fgetl(fid);
    count = count+1;
end
