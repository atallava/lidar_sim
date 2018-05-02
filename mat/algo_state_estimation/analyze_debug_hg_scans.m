% both sim scans looked really sparse. why was this the case?
sectionId = 4;
scansVersion = '300118';
dataSource = 'hg_sim';
sourceVersion = '080917';

%% rel path helpers
genRelPathEllipsoidModels = @(sectionId,sourceVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_ellipsoid_models', ...
    sectionId,sourceVersion);

genRelPathTriModels = @(sectionId,sourceVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_ground_triangles', ...
    sectionId,sourceVersion,sectionId,blockId);

%% load scans
relPathDir = genRelPathPacketsDir(sectionId, scansVersion, dataSource, sourceVersion);
relPathDir = [relPathDir '/debug'];

% real pts
relPathRealPts = [relPathDir '/real_pts.xyz'];
ptsReal = loadPts(relPathRealPts);

% sim pts
relPathSimPts = [relPathDir '/sim_pts.xyz'];
ptsSim = loadPts(relPathSimPts);

% sim detail
relPathSimDetail = [relPathDir '/sim_detail.txt'];
simDetail = loadSimDetail(relPathSimDetail);

%% hack load scans
load('hg_sim_debug', 'ptsReal', 'ptsSim', 'simDetail');

%% load models
sectionId = 4;

% ellipsoid models
% exploiting that at time of creation via mat, all ellipsoids were saved
relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId, sourceVersion);
load(relPathEllipsoidModels, 'ellipsoidModels');

% triangles
relPathModelsDir = genRelPathHgModelsDirMat(sectionId,sourceVersion);
triBlockIds = getTriModelBlockIds(relPathModelsDir,sectionId);
nTriBlocks = length(triBlockIds);
triModelCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = triBlockIds(i);
    
    relPathTriModels = genRelPathTriModels(sectionId,sourceVersion,blockId);
    container = load(relPathTriModels,'triModels');
    triModelCell{i} = container.triModels;
end
triModels = stitchTriModels(triModelCell);

%% viz real pts
hfig = figure();
scatter3(ptsReal(:,1), ptsReal(:,2), ptsReal(:,3), '.');
axis equal; 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

%% viz sim pts
hfig = figure();
scatter3(ptsSim(:,1), ptsSim(:,2), ptsSim(:,3), '.');
axis equal; 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

%% viz sim pts and models
modelNbrRadius = 4.0;
% triModelsNbr = createTriModelsNbr(triModels, ptsSim, modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels, ptsSim(1:2:end,:), modelNbrRadius);

hfig = figure();
scatter3(ptsSim(:,1), ptsSim(:,2), ptsSim(:,3), '.');
axis equal; 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
drawEllipsoids(hfig,ellipsoidModelsNbr);
% drawTriModels(hfig,triModelsNbr,'ground');

%% viz real pts and models 
modelNbrRadius = 4.0;
triModelsNbr = createTriModelsNbr(triModels, ptsReal, modelNbrRadius);
% ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels, ptsReal, modelNbrRadius);

hfig = figure();
scatter3(ptsReal(:,1), ptsReal(:,2), ptsReal(:,3), '.');
axis equal; 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
% drawEllipsoids(hfig,ellipsoidModelsNbr);
drawTriModels(hfig,triModelsNbr,'ground');









