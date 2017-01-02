function [packetIds,readTimes] = readPacketIds(pathPacketIds)
fid = fopen(pathPacketIds,'r');
packetIds = [];
readTimes = [];
nIds = 0;
line = fgetl(fid);
while(ischar(line))
    c = strsplit(line);
    nIds = nIds+1;
    packetIds(nIds) = str2num(c{1});
    readTimes(nIds) = str2num(c{2});
    line = fgetl(fid);
end
end