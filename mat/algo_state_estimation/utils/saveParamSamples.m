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
    
    % write a line. this is modified by me each time i want to change what
    % the sampled parameters are? messy.
    fprintf(fid, '%d ', sample(1)); % max_iters
    fprintf(fid, '%.6f ', sample(2)); % max_correspond_dist
    fprintf(fid, '%d ', sample(3)); % ransac_iters
    fprintf(fid, '%.6f ', sample(4)); % ransac_threshold
    fprintf(fid, '%.6f ', sample(5)); % log_min_transform_exp
    fprintf(fid, '%.6f ', sample(6)); % log_min_objective_exp
    fprintf(fid, '%.6f ', sample(7)); % log_voxel_size
    if (length(sample) == 8)
        fprintf(fid, '%d', sample(8)); % enable_guess_disps
    else
        fprintf(fid, '%d ', sample(8)); % enable_guess_disps
        fprintf(fid, '%08d', sample(9)); % guess_disps_version
    end

    if (i < size(paramSamples,1))
        fprintf(fid, '\n');
    end
end
fclose(fid);
end