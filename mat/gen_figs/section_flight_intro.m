%% load section pts
relPathSectionPts = '../data/section_pts_03_world_frame_subsampled_1e5';
load(relPathSectionPts,'pts');

%%
relPathOutMat = '../data/section_03_camera_transfs';
load(relPathOutMat,'tStart','tKeypoints','cameraPositions','viewDirns');

%%
cameraPositionsIntro = [1.0e+03 *[-0.4809    0.4438    2.5819];
1.0e+03 *[-0.4809    0.4438    2.5819];
1.0e+03 *[-0.4809    0.4438    2.5819];
1.0e+03 *[-0.3868    0.2482    2.5728];    
1.0e+03 *[0.0635   -0.7507    2.2253];
1.0e+03 *[0.2663   -1.1959    1.8539];
1.0e+03 *[0.1739   -1.4688    1.6133];
1.0e+03 *[0.1318   -1.6789    1.3471];
1.0e+03 *[-0.0024   -1.2858    1.0870];
1.0e+03 *[-0.0583   -1.1205    0.9781];
1.0e+03 *[0.0082   -1.1810    0.8400];
1.0e+03 *[0.0310   -1.2525    0.6713];
1.0e+03 *[-0.0407   -1.0557    0.5909];
-126.3701 -825.7265  493.2512;
-188.7648 -658.3909  422.3109;
-140.8162 -676.3778  327.0930;
-84.8418 -664.4469  294.7784;
-25.0328 -658.2515  213.0304;
-62.6413 -584.5153  197.6545;
-129.5819 -458.1499  170.0784;
-122.7131 -472.6811   35.2837;
-161.7485 -395.2320   30.6912;
];

cameraTargetsIntro = [-480.8832  443.7684  -12.0175;
 -480.8832  443.7684  -12.0175;   
-480.8832  443.7684  -12.0175;
-480.8832  443.7684  -12.0175;
-480.8832  443.7684  -12.0175;
-480.8832  443.7684  -12.0175;
-480.8832  443.7684  -12.0175;
-480.8832  443.7684  -12.0175;
-497.7962  430.5977  -12.0175;
-504.6552  425.8336  -12.0175;
-504.6552  425.8336  -12.0175;
-504.6552  425.8336  -12.0175;
-513.3343  425.2446  -12.0175;
-522.4222  415.2540  -12.0175;
-529.2107  408.3545  -12.0175;
-529.2107  408.3545  -12.0175;
-529.2107  408.3545  -12.0175;
-529.2107  408.3545  -12.0175;
-532.3722  409.2170  -12.0175;
-538.0436  405.9652  -12.2450;
-538.0436  405.9652  -12.2450;
-540.4693  405.9652  -12.6480;
];

viewDirnsIntro = calcViewDirns(cameraPositionsIntro,cameraTargetsIntro);

%% viz
% plot intro + path on points
scatter3(pts(:,1),pts(:,2),pts(:,3),'b.');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); 
hold on;
% plot3(cameraPositions(:,1),cameraPositions(:,2),cameraPositions(:,3),'r','linewidth',5);
% plot3(cameraPositionsIntro(:,1),cameraPositionsIntro(:,2),cameraPositionsIntro(:,3),'g','linewidth',5);
% smash together camera positions
cameraPositionsAll = [cameraPositions; cameraPositionsIntro];
plot3(cameraPositionsAll(:,1),cameraPositionsAll(:,2),cameraPositionsAll(:,3),'r','linewidth',5);




