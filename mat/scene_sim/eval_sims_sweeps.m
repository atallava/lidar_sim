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

%% hg
relPathHgSimDetail = genRelPathHgSimDetail(sectionId,tag);
load(relPathHgSimDetail,'simDetail');
[errVecHg,missVecHg,detailIdsHg] = getSweepError(simDetail,ptsGroundRef);

%% mm
relPathMmSimDetail = genRelPathMmSimDetail(sectionId,tag);
load(relPathMmSimDetail,'simDetail');
[errVecMm,missVecMm,detailIdsMm] = getSweepError(simDetail,ptsGroundRef);

%% plot
figure;
plot(detailIdsHg,errVecHg,'r');
hold on;
plot(detailIdsMm,errVecMm,'b');
xlabel('detail ids');
ylabel('err');
legend('hg','mm');

figure;
plot(detailIdsHg,errVecMm-errVecHg,'-+b');
hold on;
plot(detailIdsHg,zeros(1,length(detailIdsHg)),'r');
xlabel('detail ids');
ylabel('delta err');

figure;
plot(detailIdsHg,missVecHg,'r');
hold on;
plot(detailIdsMm,missVecMm,'b');
xlabel('detail ids');
ylabel('misses');
legend('hg','mm');

figure;
plot(detailIdsHg,missVecMm-missVecHg,'-+b');
hold on;
plot(detailIdsHg,zeros(1,length(detailIdsHg)),'r');
xlabel('detail ids');
ylabel('delta miss');

%%
nParts = 5;
nodes = floor(linspace(1,length(detailIdsHg),nParts+1));
plotIdsCell = cell(1,nParts);
for i = 1:nParts
    plotIdsCell{i} = nodes(i):(nodes(i+1)-1);
end
part = 5;
plotIds = plotIdsCell{part};

dErr = errVecMm-errVecHg;
dMiss = missVecMm-missVecHg;

figure;
plot(detailIdsHg(plotIds),dErr(plotIds),'-+b');
hold on;
plot(detailIdsHg(plotIds),zeros(1,length(plotIds)),'r');
xlabel('detail ids');
ylabel('delta err');

figure;
plot(detailIdsHg(plotIds),dMiss(plotIds),'-+b');
hold on;
plot(detailIdsHg(plotIds),zeros(1,length(plotIds)),'r');
xlabel('detail ids');
ylabel('delta miss');


