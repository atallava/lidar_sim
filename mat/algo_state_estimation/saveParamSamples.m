function saveParamSamples(relPathSamples, paramSamples)
%SAVEPARAMSAMPLES
%
% SAVEPARAMSAMPLES(relPathSamples, paramSamples)
%
% relPathSamples -
% paramSamples   -

fid = fopen(relPathSamples, 'w');
for i = 1:size(paramSamples,1)
    sample = paramSamples(i,:);
    for j = 1:length(sample)
        fprintf(fid, '%.6f', sample(j));
        if j < length(sample)
            fprintf(fid, ' ');
        end
    end
    if (i < size(paramSamples,1))
        fprintf(fid, '\n');
    end
end
fclose(fid);
end