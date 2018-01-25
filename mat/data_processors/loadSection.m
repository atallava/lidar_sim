function section = loadSection(relPathSection,numPacketsToRead)
%LOADSECTION warning: don't try reading too many packets, matlab will freeze
%
% section = LOADSECTION(relPathSection,numPacketsToRead)
%
% relPathSection   - string.
% numPacketsToRead - int. defaults to 5.
%
% section          - struct.

if nargin < 2
    numPacketsToRead = 5;
end

fid = fopen(relPathSection,'r');
section.pts = [];
section.ptTimestamps = [];
section.packetIds = [];
section.packetTimestamps = [];

packetsRead = -1;
prevPacketId = -1;

line = fgetl(fid);

while ischar(line)
    c = strsplit(line);
    packetId = str2double(c{1});
    packetTimestampSec = str2double(c{2});
    packetTimestampNanoSec = str2double(c{3});
    packetTimestamp = packetTimestampSec + packetTimestampNanoSec*1e-9;
    x = str2double(c{4});
    y = str2double(c{5});
    z = str2double(c{6});
    pt = [x y z];
    
    if packetId ~= prevPacketId
        packetsRead = packetsRead+1;
        if packetsRead >= numPacketsToRead
            break;
        end
        section.packetIds = [section.packetIds packetId];
        section.packetTimestamps = [section.packetTimestamps packetTimestamp];
    end
    prevPacketId = packetId;
    
    section.pts = [section.pts; pt];
    section.ptTimestamps = [section.ptTimestamps packetTimestamp];
    
    line = fgetl(fid);
end
end