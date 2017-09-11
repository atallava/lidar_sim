% random pts + cov
nPts = 4;
pts = 10*rand(nPts,3);
% pts = [ 1,2,3;
% 	6,7,6;
% 	5,1,5;
% 	9,7,8];
meanPts = mean(pts,1);
covMat = cov(pts);

%% plot
[x1,y1,z1] = genSurfXyzEllipse(covMat,meanPts);
surf(x1,y1,z1,'facecolor','b','facealpha',0.1,'meshstyle','none');
hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3));
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');


%% plot
figure;
[x2,y2,z2] = genSurfXyzEllipse2(covMat,meanPts);
surf(x2,y2,z2,'facecolor','b','facealpha',0.1,'meshstyle','none');
hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3));
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');



