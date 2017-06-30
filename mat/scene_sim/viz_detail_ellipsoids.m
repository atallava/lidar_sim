%% rel path helpers
genRelPathHgSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/hg_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathGroundPts = @(sectionId) ...
    sprintf('../data/sections/section_%02d/ground_segmentation/section_pts_%02d_ground', ...
    sectionId,sectionId);

%% load 
sectionId = 4;
tag = 1;

% ground pts
relPathGroundPts = genRelPathGroundPts(sectionId);
load(relPathGroundPts,'pts');
ptsGroundRef = pts;

% ellipsoids
relPathEllipsoids = '../data/sections/section_04/hg_sim/object_ellipsoid_models.mat';
load(relPathEllipsoids,'ellipsoidModels');

%%
relPathHgSimDetail = genRelPathHgSimDetail(sectionId,tag);
load(relPathHgSimDetail,'simDetail');
simDetailHg = simDetail;

[membershipIdsHg,ptsRealHg,ptsSimHg,hitFlagHg] = unrollSimDetail(simDetailHg);

thresh = 5e-3;
[ids,dists] = knnsearch(ptsGroundRef,ptsRealHg);
nonGroundFlag = (dists > thresh);
nonGroundFlag = flipVecToRow(nonGroundFlag);

%%
detailIds = 21801:1:22701;

[pts1,pts2] = deal([]);
for i = 1:length(detailIds)
    detailId = detailIds(i);
    ptIds = find(membershipIdsHg == detailId);

    flag = nonGroundFlag(ptIds);
    ptIdsReal = ptIds(flag);
    pts1 = [pts1; ptsRealHg(ptIdsReal,:)];
    
    flag = hitFlagHg(ptIds) & nonGroundFlag(ptIds);
    ptIdsHg = ptIds(flag);
    pts2 = [pts2; ptsSimHg(ptIdsHg,:)];
end

%% 
modelNbrRadius = 3;

box = [-126.5950  -74.5619; 272.7490  313.7880];
ptsQuery = getPtsInBox(pts2,box);

ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,ptsQuery,modelNbrRadius);

%%
plotStructVars = {'rayData','ellipsoidData','triModelData','pts','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModelsNbr;
ellipsoidData.uniformAlpha = true;
plotStruct.ellipsoidData = ellipsoidData;

hfig = plotRangeData(plotStruct);

%%
figure(hfig); hold on;
marker = '.';
markerSize = 50;
scatter3(pts1(:,1),pts1(:,2),pts1(:,3),'marker',marker,'sizeData',markerSize,'markerEdgeColor','r');
scatter3(pts2(:,1),pts2(:,2),pts2(:,3),'marker',marker,'sizeData',markerSize,'markerEdgeColor','b');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

%% view
view(2);
xlim([-126.5950  -74.5619]);
ylim([272.7490  313.7880]);
clear('box');
box on;
set(gca,'fontsize',15);

