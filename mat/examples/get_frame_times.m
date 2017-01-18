relPathFrameTimestamp = '../data/taylorJune2014/lowres/AVTCamera_156580/Camera_156580_Timestamp.txt';
clockLocal = tic();
[frameIdLog,frameTLog] = processFrameTimestamp(relPathFrameTimestamp);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%%
relPathOutput = 'frame_times';
save(relPathOutput,'frameIdLog','frameTLog');
