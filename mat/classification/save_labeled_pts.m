relPathLabeledPts = '../data/sections/section_03/classification/pts_ground_truth.mat';
load(relPathLabeledPts,'pts','labels');

%%
% relPathOut = '../../cpp/data/sections/section_03/classification/pts_labeled.txt';
relPathOut = 'trial.txt';

clockLocal = tic();
fid = fopen(relPathFile,'w');
for i = 1:size(pts,1)
    if ~labels(j)
        continue;
    end
    pt = pts(i,:);
    line = '';
    for j = 1:length(pt)
        line = [line ' ' num2str(pt(j))];
    end
    line = [line ' ' num2str(labels(j))];
    % remove trailing whitespaces
    line = strtrim(line);
    line = sprintf('%s\n',line);
    fprintf(fid,line);
end
fclose(fid);
elapsedTime = toc(clockLocal);

fprintf('elapsed time: %.2f\n',elapsedTime);