function hfig = genFigPrimitive(T_segment_to_world,pts,obb,ellipsoidModels,visibility)
%GENFIGPRIMITIVE
%
% hfig = GENFIGPRIMITIVE(T_segment_to_world,pts,obb,ellipsoidModels,visibility)
%
% T_segment_to_world - [4,4] array. Transform.
% pts                - [nPts,3] array. In Id frame.
% obb                - struct. In Id frame.
% ellipsoidModels    - struct array.
% visibility         - string. default 'on'.
%
% hfig               - figure handle.

if nargin < 5
    visibility = 'on';
end

% transform back to world frame
pts_world = applyTransf(pts,T_segment_to_world);
obb_world = applyTransfToObb(obb,T_segment_to_world);
ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,T_segment_to_world);

% plot
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});
ellipsoidData.ellipsoidModels = ellipsoidModels_world;
plotStruct.ellipsoidData = ellipsoidData;
hfig = plotRangeData(plotStruct);
set(hfig,'visible',visibility);
drawObb(hfig,obb_world,pts_world);
end