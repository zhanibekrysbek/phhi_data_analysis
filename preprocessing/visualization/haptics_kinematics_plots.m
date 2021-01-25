

% clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);
% observations_processed = adjust_time(observations);

%% Force Plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/force/body/';

figure('visible','off');
for ind=progress(1:numel(observations_processed),'Title', 'Force Plots')
    obs = observations_processed(ind);
    plot_rfts(obs,1)
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


%% Force Vector Plots
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


fig_path = '../../data/plots/plots_preprocessed_v2_1/trajectory_vector_body_fr/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'TrajVectorPlot')
    
    obs = observations_processed(ind);

    trajvector_plot(obs,2)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])

end

%% Phase Plot


fig_path = '../../data/plots/plots_preprocessed_v2_1/rft_accel_phase/all_axis/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'PhasePlot')

    obs = observations_processed(ind);

    plot_phase(obs,3)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])

end

%% IMU Plot

fig_path = '../../data/plots/plots_preprocessed_v2_1/imu/all_dofs';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'IMUplot')

    obs = observations_processed(ind);

    plot_imu(obs,1)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])

end

%% IMU and ARUCO Orientation Comparison plot
fig_path = '../../data/plots/plots_preprocessed_v2_1/orient_tracking/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'OrientTracking_plot')

    obs = observations_processed(ind);

    plot_imu(obs, 2)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])

end
figure('visible','on');


%% Twist
fig_path = '../../data/plots/plots_preprocessed_v2_1/twist/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'Twist')

    obs = observations_processed(ind);

    plot_imu(obs, 3)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])

end
figure('visible','on');


%% Accel
fig_path = '../../data/plots/plots_preprocessed_v2_1/imu/accel/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'Accel')

    obs = observations_processed(ind);

    plot_imu(obs, 4)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])

end
figure('visible','on');


%%

obs = observations_processed(3);

% figure(1)
% plot_phase(obs,1);
% figure(5)
% plot_rfts(obs,2);
% figure(3)
% plot_imu(obs,1);

figure(4);
plot_imu(obs,4)






















