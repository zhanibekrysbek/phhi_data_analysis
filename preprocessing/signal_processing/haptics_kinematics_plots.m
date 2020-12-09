

clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);

observations_processed = adjust_time(observations_processed);

%% Force Plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/force/';

figure('visible','off');
for ind=progress(1:numel(observations_processed),'Title', 'Force Plots')
    obs = observations_processed(ind);
    plot_rfts(obs,2)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'force', '.jpg'])
end


%% Kinematics and Stretch Force plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/kinematics_fstretch/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'Kinematics_Stretch')
    
    obs = observations_processed(ind);
%     try
    plot_pose(obs,3)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])
%     catch
%         fprintf('1 error is found! \n')
%         fails = [fails;ind];
%     end
    
end


%% Kinematics and Orientation plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/kinematics_orientation/';
figure('visible','off');
for ind=progress(1:numel(observations_processed),'Title', 'Kinematics_Orient')
    
    obs = observations_processed(ind);
    plot_pose(obs,2)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])
end
%%




