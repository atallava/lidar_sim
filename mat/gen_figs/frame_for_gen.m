%% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

genRelPathFrame = @(frameId) ...
    sprintf('../../data/taylorJune2014/lowres/AVTCamera_156580/Camera_156580_%06d.jpg',frameId);

%% 
id1 = 9400;
fname1 = genRelPathFrame(id1);
frame1 = imread(fname1);
% magic
frame1 = imadjust(frame1, [0 0 0; 1 1 1]*0.4,[]);
image(frame1);

