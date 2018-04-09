function hfig = vizEllipsoids(ellipsoidModels)
%VIZELLIPSOIDS
%
% hfig = VIZELLIPSOIDS(ellipsoidModels)
%
% ellipsoidModels - struct.
%
% hfig            - figure handle.

hfig = figure();
drawEllipsoids(hfig, ellipsoidModels);
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
end