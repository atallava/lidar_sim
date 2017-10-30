function hfig = genFigPrimitivePatch(cellT_segment_to_world,cellPts,cellObb,cellEllipsoidModels,visibility)
%GENFIGPRIMITIVEPATCH
%
% hfig = GENFIGPRIMITIVEPATCH(cellT_segment_to_world,cellPts,cellObb,cellEllipsoidModels,visibility)
%
% cellT_segment_to_world - cell array.
% cellPts                - cell array.
% cellObb                - cell array.
% cellEllipsoidModels    - cell array.
% visibility             - string. default 'on'.
%
% hfig                   - figure handle.

if nargin < 5
    visibility = 'on';
end

nCells = length(cellT_segment_to_world);
[cellObbs_world,cellPts_world,cellEllipsoidModels_world] = deal(cell(1,nCells));

for i = 1:nCells
    T_segment_to_world = cellT_segment_to_world{i};
    pts = cellPts{i};
    obb = cellObb{i};
    ellipsoidModels = cellEllipsoidModels{i};
    
    % transform back to world frame
    cellPts_world{i} = applyTransf(pts,T_segment_to_world);
    cellObbs_world{i} = applyTransfToObb(obb,T_segment_to_world);
    cellEllipsoidModels_world{i} = applyTransfToEllipsoids(ellipsoidModels,T_segment_to_world);
end
patchEllipsoidModels_world = stitchEllipsoidModels(cellEllipsoidModels_world);

% figure
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});
ellipsoidData.ellipsoidModels = patchEllipsoidModels_world;
plotStruct.ellipsoidData = ellipsoidData;
hfig = plotRangeData(plotStruct);
set(hfig,'visible',visibility);
% draw all obbs
for i = 1:nCells
    drawObb(hfig,cellObbs_world{i},cellPts_world{i});
end
            
end