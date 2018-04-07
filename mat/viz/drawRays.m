function drawRays(hfig, rayOrigins, rayDirns)
%DRAWRAYS
%
% DRAWRAYS(hfig, rayOrigins, rayDirns)
%
% hfig       - figure handle.
% rayOrigins - length 3 vector, or [nRays,3] array.
% rayDirns   - [nRays,3] array.

nRays = size(rayDirns,1);
if isvector(rayOrigins)
    rayOrigins = flipVecToRow(rayOrigins);
    rayOrigins = repmat(rayOrigins, nRays, 1);
end

figure(hfig);
for i = 1:nRays
    pts = genPtsRay(rayOrigins(i,:), rayDirns(i,:), 10);
    plot3(pts(:,1),pts(:,2),pts(:,3),'g--');
    plot3(rayOrigins(i,1), rayOrigins(i,2), rayOrigins(i,3),'gx');
    hold on;
    axis equal;
end
end