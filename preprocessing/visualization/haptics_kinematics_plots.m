

% clc;clear;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

%% Force Plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/force/';

figure('visible','off');
for ind=progress(1:numel(observations_processed),'Title', 'Force Plots')
    obs = observations_processed(ind);
    
    plot_rfts(obs,1)
    saveas(gcf, [fig_path, 'body/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'force_body', '.jpg'])
    
    plot_rfts(obs,2)
    saveas(gcf, [fig_path, 'spatial/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'force_spatial', '.jpg'])
end


%% Kinematics and Stretch Force plots

fig_path = '../../data/plots/plots_preprocessed_v2_1/kinematics2D/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'Kinematics_Stretch')
    
    obs = observations_processed(ind);

    plot_pose(obs,2)
    saveas(gcf, [fig_path, 'orientation/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics_fstretch', '.jpg'])
    
    plot_pose(obs,3)
    saveas(gcf, [fig_path,'fstretch/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics_orient', '.jpg'])
    
    plot_pose(obs,4)
    saveas(gcf, [fig_path,'mix_plot/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics_orient', '.jpg'])
    
    
end


%% Force Vector Plots - Animation
obs = observations_processed(10);

figure(1);

for ind = 1:20:numel(obs.rft1.time_steps)
    ip = 1+round(ind/10);
    plotv(obs.rft1.forceS(ind,1:2)','-->r'); hold on;
    plotv(obs.rft2.forceS(ind,1:2)','-->g')
    plotv(obs.fsum.forceS(ind,1:2)','->b')
    plotv(5*obs.pose123.linvel(ip,1:2)','-.xc')
    
    hold off; grid on;
    legend('F_1', 'F_2', 'F_{sum}', 'vel');
    title(sprintf('%2.2d sec  %2.2d sec', obs.rft1.time_steps(ind), obs.pose123.time_steps(ip)))
    xlim([-10,10]);
    ylim([-10,10]);
    
    pause(0.01);
end

    
%% Quiver Vector Plot Function


fig_path = '../../data/plots/plots_preprocessed_v2_1/kinematics2D/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'TrajVectorPlot')
    
    obs = observations_processed(ind);

%     trajvector_plot(obs,1)
%     saveas(gcf, [fig_path, 'trajvector_spatial/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'trajVector_spatial', '.jpg'])
%     
%     trajvector_plot(obs,2)
%     saveas(gcf, [fig_path, 'trajvector_body/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'trajVector_body', '.jpg'])

    trajvector_plot(obs,3)
    saveas(gcf, [fig_path, 'trajvector_spatial_tdec/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'trajVector_body', '.jpg'])

end


%% Phase Plot


fig_path = '../../data/plots/plots_preprocessed_v2_1/rft_accel_phase/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'PhasePlot')

    obs = observations_processed(ind);

    plot_phase(obs,1)
    saveas(gcf, [fig_path, 'mag_phase/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'mag_phase', '.jpg'])
    
    plot_phase(obs,3)
    saveas(gcf, [fig_path, 'all_axis/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'all_axis', '.jpg'])


end


%% IMU Plot

fig_path = '../../data/plots/plots_preprocessed_v2_1/imu/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'IMUplot')

    obs = observations_processed(ind);

    plot_imu(obs,1)
    saveas(gcf, [fig_path, 'all_dofs/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'imu', '.jpg'])

    plot_imu(obs, 4)
    saveas(gcf, [fig_path, 'accel/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'accel', '.jpg'])

    plot_imu(obs, 2)
    saveas(gcf, [fig_path, 'orient_tracking/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'orient_tracking', '.jpg'])

end



%% Twist
fig_path = '../../data/plots/plots_preprocessed_v2_1/twist/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'Twist')

    obs = observations_processed(ind);

    plot_imu(obs, 3)
    saveas(gcf, [fig_path, 'spatial/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'twist_spatial', '.jpg'])
    
    plot_imu(obs, 5)
    saveas(gcf, [fig_path, 'body/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'twist_body', '.jpg'])
end
figure('visible','on');


%% Events
fig_path = '../../data/plots/plots_preprocessed_v2_1/events/';
% features = haptic_features(observations_processed);

st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);

figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'Events')

    obs = observations_processed(ind);
    feat = features(ind);
    win_locs = M(obs.obs_id);
    
    plot_events(obs, feat, win_locs, 1)
    saveas(gcf, [fig_path, '/mixplot/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'events', '.jpg'])
    plot_events(obs, feat, win_locs, 2)
    saveas(gcf, [fig_path, '/angles/', obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'events', '.jpg'])

end
figure('visible','on');

%%
% features = haptic_features(observations_processed);

st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);

ind = 5;
obs = observations_processed(ind);
% obs.tdec_sec = 5;
feat = features(ind);
win_locs = M(obs.obs_id);

figure(ind);
plot_events(obs, feat, win_locs, 4)


%%

for ind = 101:112
    
st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);

obs = observations_processed(ind);
feat = features(ind);
win_locs = M(obs.obs_id);

figure(ind);
plot_events(obs, feat, win_locs, 1)

end

% figure(7);
% plot_events(obs, feat, win_locs, 2)



% figure(1)
% plot_phase(obs,1);
% figure(5)
% plot_rfts(obs,2);
% figure(3)
% plot_imu(obs,1);

% figure(4);
% trajvector_plot(obs,3)
% 
% figure(5);
% plot_pose(obs,4)










