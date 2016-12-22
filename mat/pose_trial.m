relPathIns = 'Pose/PoseAndEncoder_1797_0000254902_wgs84.mat';
load(relPathIns);
insPP = INS_PP;

%% 
dataId = 10;

names = fieldnames(insPP);
for i = 1:length(names)
    name = names{i};
    var = insPP.(name);
    fprintf('%s: %s\n',name,num2str(var(dataId)));
end

%%
relPathFixed = 'Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed';
fidFixed = fopen(relPathFixed,'r');

%%
count = 0;
while count < dataId
    line = fgetl(fidFixed);
    count = count+1;
end
c = strsplit(line);
disp(c);

%%
poses = getPosesFromFixed(relPathFixed);