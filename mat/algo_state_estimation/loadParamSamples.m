function paramSamples = loadParamSamples(relPathParamSamples)
%LOADPARAMSAMPLES
%
% paramSamples = LOADPARAMSAMPLES(relPathParamSamples)
%
% relPathParamSamples -
%
% paramSamples        -

fid = fopen(relPathParamSamples, 'r');
paramSamples = [];
count = 1;
line = fgetl(fid);
while ischar(line)
    line = strtrim(line);
    c = strsplit(line);
    sample = zeros(1,length(c));
    for i = 1:length(c)
        sample(i) = str2double(c{i});
    end
    paramSamples(count,:) = sample;
    count = count+1;
    line = fgetl(fid);
end
end