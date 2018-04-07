function hfig = vizRays(origins, rayDirns)
%VIZRAYS
%
% hfig = VIZRAYS(origin,rayDirns)
%
% origin   - length 3 vector. or [nRays,3] array.
% rayDirns - [nRays,3] array.
%
% hfig     - figure handle.

hfig = figure();
drawRays(hfig, origins, rayDirns);
end