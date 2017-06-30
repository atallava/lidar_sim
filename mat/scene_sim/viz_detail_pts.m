genRelPathHgSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/hg_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathMmSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/mesh_model_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathGroundPts = @(sectionId) ...
    sprintf('../data/sections/section_%02d/ground_segmentation/section_pts_%02d_ground', ...
    sectionId,sectionId);

%%
sectionId = 4;
tag = 1;

relPathGroundPts = genRelPathGroundPts(sectionId);
load(relPathGroundPts,'pts');
ptsGroundRef = pts;

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
relPathMmSimDetail = genRelPathMmSimDetail(sectionId,tag);
load(relPathMmSimDetail,'simDetail');
simDetailMm = simDetail;

[membershipIdsMm,ptsRealMm,ptsSimMm,hitFlagMm] = unrollSimDetail(simDetailMm);

%%
detailIds = 40001:40301;

[pts1,pts2,pts3] = deal([]);
for i = 1:length(detailIds)
    detailId = detailIds(i);
    ptIds = find(membershipIdsHg == detailId);

    flag = nonGroundFlag(ptIds);
    ptIdsReal = ptIds(flag);
    pts1 = [pts1; ptsRealHg(ptIdsReal,:)];
    
    flag = hitFlagHg(ptIds) & nonGroundFlag(ptIds);
    ptIdsHg = ptIds(flag);
    pts2 = [pts2; ptsSimHg(ptIdsHg,:)];
    
    flag = hitFlagMm(ptIds) & nonGroundFlag(ptIds);
    ptIdsMm = ptIds(flag);
    pts3 = [pts3; ptsSimMm(ptIdsMm,:)];
end

%%
figure; hold on;
scatter3(pts1(:,1),pts1(:,2),pts1(:,3),'r');
scatter3(pts2(:,1),pts2(:,2),pts2(:,3),'b');
scatter3(pts3(:,1),pts3(:,2),pts3(:,3),'g');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('real','hg','mm');

