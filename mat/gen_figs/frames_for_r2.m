%% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

genRelPathFrame = @(frameId) ...
    sprintf('../../data/taylorJune2014/lowres/AVTCamera_156580/Camera_156580_%06d.jpg',frameId);

%% 
id1 = 8640;
fname1 = genRelPathFrame(id1);
frame1 = imread(fname1);
% magic
frame1 = imadjust(frame1, [0 0 0; 0.25 0.25 0.25],[]);
image(frame1);

%% 
id2 = id1+50;
fname2 = genRelPathFrame(id2);
frame2 = imread(fname2);
% magic
frame2 = imadjust(frame2, [0 0 0; 0.25 0.25 0.25],[]);
image(frame2);