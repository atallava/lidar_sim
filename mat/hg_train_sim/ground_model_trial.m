relPathTrainPts = '../data/rim_stretch_ground_train.mat';
load(relPathTrainPts,'pts');

relPathModelingParams = '../data/modeling_params';
load(relPathModelingParams,'modelingParams');

triParams = modelingParams.triParams;

%% fit smoothed pts
ptsFit = getSmoothedFitToGroundPts(pts,triParams);

%% viz pts fit
figure;
subplot(2,1,1);
scatter3(ptsFit(:,1),ptsFit(:,2),ptsFit(:,3),'r.');
axis equal;

subplot(2,1,2);
scatter3(ptsFit(:,1),ptsFit(:,2),ptsFit(:,3),'r.');
hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3),'b.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');

%% delaunay 
tri = delaunay(ptsFit(:,1),ptsFit(:,2));

%% viz
figure;
subplot(2,1,1);
trimesh(tri,ptsFit(:,1),ptsFit(:,2),ptsFit(:,3));
axis equal;

subplot(2,1,2);
trimesh(tri,ptsFit(:,1),ptsFit(:,2),ptsFit(:,3));
hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3),'b.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');

%% write
groundTriModel.tri = tri;
groundTriModel.ptsFit = ptsFit;

relPathGroundModel = '../data/ground_model';
save(relPathGroundModel,'groundTriModel');