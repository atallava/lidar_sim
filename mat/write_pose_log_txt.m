relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog','tOffset');

%%
relPathOut = 'pose_log.txt';
fid = fopen(relPathOut,'w');
nPoses = size(poseLog,1);
for i = 1:nPoses
    line = sprintf('%f %f %f %f %f %f %f\n',...
        tLog(i)+tOffset, ...
        poseLog(i,1),poseLog(i,2),poseLog(i,3), ...
        poseLog(i,4),poseLog(i,5),poseLog(i,6));
    fprintf(fid,line);
end