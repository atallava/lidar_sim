% genRelPathPdf = @(fname) sprintf('~/thesis_proposal/tex/figs/offroad_lidar_sim/%s.pdf',fname);
% genRelPathJpg = @(fname) sprintf('~/thesis_proposal/tex/figs/jpgs/%s.jpg',fname);
% 
% fnames = {'algo_dev_path','algo_dev_real','algo_dev_sim_1','algo_risk_sim_1','algo_risk_sim_2','algo_risk_sim_2_extra', ...
%     'classes_in_frame','ellipsoid_intersections','ellipsoid_models','flowchart_sim','flowchart_sim_color','flowchart_train',...
%     'flowchart_train_color','gantt','map','map_unannotated','map_with_pcd','primitives','scene_sim','sim_star_objective_gap','single_scene_sim',...
%     'sparser_state_mapping','triangle_mesh_model','voxnet_data_cartoon'};

genRelPathPdf = @(fname) sprintf('~/thesis_proposal/ppt/figs/sim_evaln/%s.pdf',fname);
genRelPathJpg = @(fname) sprintf('~/thesis_proposal/ppt/figs/sim_evaln/jpgs/%s.jpg',fname);

dirRes = dir('~/thesis_proposal/ppt/figs/sim_evaln');
fnames = {};
for i = 1:length(dirRes)
    posn = strfind(dirRes(i).name,'pdf');
    if (isempty(posn) ~= 1)
        [~,fnames{end+1},~] = fileparts(dirRes(i).name);
    end
end

%%
for i = 1:length(fnames)
    fname = fnames{i};
    cmd = sprintf('convert -density 300 -quality 100 %s %s',genRelPathPdf(fname),genRelPathJpg(fname));
    system(cmd);
end