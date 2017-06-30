%% rel path helpers
genRelPathMmSimDetail = @(sectionId,tag) ...
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
relPathTriModels = '../data/sections/section_04/mesh_model_sim/scene_tri_models.mat';
load(relPathTriModels,'sceneTriModels');

%%
relPathMmSimDetail = genRelPathMmSimDetail(sectionId,tag);
load(relPathMmSimDetail,'simDetail');
simDetailMm = simDetail;

[membershipIdsMm,ptsRealMm,ptsSimMm,hitFlagMm] = unrollSimDetail(simDetailMm);

thresh = 5e-3;
[ids,dists] = knnsearch(ptsGroundRef,ptsRealMm);
nonGroundFlag = (dists > thresh);
nonGroundFlag = flipVecToRow(nonGroundFlag);

%%
detailIds = 12701:13301;

[pts1,pts2] = deal([]);
for i = 1:length(detailIds)
    detailId = detailIds(i);
    ptIds = find(membershipIdsMm == detailId);

    flag = nonGroundFlag(ptIds);
    ptIdsReal = ptIds(flag);
    pts1 = [pts1; ptsRealMm(ptIdsReal,:)];
    
    flag = hitFlagMm(ptIds) & nonGroundFlag(ptIds);
    ptIdsMm = ptIds(flag);
    pts2 = [pts2; ptsSimMm(ptIdsMm,:)];
end

%% 
modelNbrRadius = 1;
sceneTriModelsNbr = createSceneTriModelsNbr(sceneTriModels,pts1,modelNbrRadius);
triModelsNbr = stitchTriModels(sceneTriModelsNbr);

%%
plotStructVars = {'rayData','ellipsoidData','triModelData','pts','plotStruct'};
clear(plotStructVars{:});

triModelData = triModelsNbr;
% triModelData.uniformAlpha = true;
plotStruct.triModelData = triModelData;

hfig = plotRangeData(plotStruct);

%%
figure(hfig); hold on;
scatter3(pts1(:,1),pts1(:,2),pts1(:,3),'r.');
scatter3(pts2(:,1),pts2(:,2),pts2(:,3),'b.');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

